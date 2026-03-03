/*
  Servo log + load detection (no PWM in log)
  Author: Alejandro Alonso Puig (https://github.com/aalonsopuig) + ChatGPT 4.1
  License: Apache-2.0
*/

#include <Arduino.h>
#include <string.h>
#include <math.h>

// ---------- Printing helpers (fixed width) ----------
static void printFloatFixed(float v, uint8_t width, uint8_t prec) {
  char buf[24];
  dtostrf(v, width, prec, buf);   // width includes sign and decimal point
  Serial.print(buf);
}

static void printIntFixed(long v, uint8_t width) {
  char buf[16];
  ltoa(v, buf, 10);
  int n = (int)strlen(buf);
  for (int i = 0; i < (int)width - n; i++) Serial.print(' ');
  Serial.print(buf);
}

// ---------- Load detection + velocity estimation ----------
struct LoadMonitorConfig {
  // Velocity estimation
  float dt_min_s        = 0.0005f;   // ignore absurdly small dt
  float ema_alpha_vreal = 0.25f;     // EMA filter for Vreal

  // "Load" detection (not stall)
  float vexp_min        = 8.0f;      // deg/s, below this -> not evaluable
  float err_min         = 4.0f;      // deg, below this -> not evaluable
  float k_follow        = 0.35f;     // if |Vreal| < k_follow*|Vexp| -> suspect load
  uint8_t load_count    = 8;         // consecutive hits to declare LOAD

  // Log decimation (optional): print every N calls
  uint8_t log_every_n   = 1;         // set to 2..10 if loop is very fast
};

struct LoadMonitorState {
  uint32_t t_prev_us = 0;
  float meas_prev    = 0.0f;
  float vreal_f      = 0.0f;
  uint8_t load_cnt   = 0;
  uint32_t n_calls   = 0;
};

static void printHeader() {
  Serial.println(" Target |    Cmd |   Meas |    Err |   Vexp |  VrealF | LIdx | Flag");
}

// Returns: true if LOAD declared. Also outputs vreal_f and loadIdx.
static bool updateLoadMonitor(
  const LoadMonitorConfig& cfg,
  LoadMonitorState& st,
  float meas, float vexp, float err,
  float& out_vreal_f,
  float& out_loadIdx
) {
  uint32_t t_us = micros();
  float dt = 0.0f;

  if (st.t_prev_us != 0) {
    uint32_t dtu = t_us - st.t_prev_us;   // unsigned handles overflow correctly
    dt = (float)dtu * 1e-6f;
  }
  st.t_prev_us = t_us;

  float vreal = 0.0f;
  if (dt > cfg.dt_min_s) {
    vreal = (meas - st.meas_prev) / dt;   // deg/s if meas is deg
  }
  st.meas_prev = meas;

  // EMA filter for Vreal
  st.vreal_f = cfg.ema_alpha_vreal * vreal + (1.0f - cfg.ema_alpha_vreal) * st.vreal_f;

  // Load criterion
  float avexp  = fabsf(vexp);
  float aerr   = fabsf(err);
  float avreal = fabsf(st.vreal_f);

  if (avexp > cfg.vexp_min && aerr > cfg.err_min) {
    if (avreal < cfg.k_follow * avexp) {
      if (st.load_cnt < 255) st.load_cnt++;
    } else {
      if (st.load_cnt > 0) st.load_cnt--;
    }
  } else {
    if (st.load_cnt > 0) st.load_cnt--;   // relax slowly when not evaluable
  }

  bool load = (st.load_cnt >= cfg.load_count);

  // Continuous index 0..1
  float loadIdx = 0.0f;
  if (avexp > cfg.vexp_min) {
    loadIdx = 1.0f - (avreal / (avexp + 1e-3f));
    if (loadIdx < 0.0f) loadIdx = 0.0f;
    if (loadIdx > 1.0f) loadIdx = 1.0f;
  }

  out_vreal_f = st.vreal_f;
  out_loadIdx = loadIdx;
  return load;
}

// Call this from your loop with your values (degrees and deg/s recommended)
static void logServoLine(
  const LoadMonitorConfig& cfg,
  LoadMonitorState& st,
  float target, float cmd, float meas, float vexp, float err
) {
  st.n_calls++;
  if (cfg.log_every_n > 1 && (st.n_calls % cfg.log_every_n) != 0) return;

  float vreal_f = 0.0f;
  float loadIdx = 0.0f;
  bool load = updateLoadMonitor(cfg, st, meas, vexp, err, vreal_f, loadIdx);

  // Fixed-width aligned log (no PWM)
  printFloatFixed(target, 7, 1); Serial.print(" | ");
  printFloatFixed(cmd,    7, 1); Serial.print(" | ");
  printFloatFixed(meas,   7, 1); Serial.print(" | ");
  printFloatFixed(err,    7, 1); Serial.print(" | ");
  printFloatFixed(vexp,   7, 1); Serial.print(" | ");
  printFloatFixed(vreal_f,8, 1); Serial.print(" | ");
  printFloatFixed(loadIdx,5, 2); Serial.print(" | ");
  Serial.println(load ? "LOAD" : "OK");
}

// ---------- Example integration ----------
// Replace these with your real variables / functions.
static LoadMonitorConfig g_cfg;
static LoadMonitorState  g_st;

void setup() {
  Serial.begin(115200);
  delay(200);
  printHeader();

  // Optional tweaks:
  // g_cfg.log_every_n = 5;          // if loop is fast, print every 5th call
  // g_cfg.ema_alpha_vreal = 0.35f;  // more smoothing
  // g_cfg.vexp_min = 10.0f;
  // g_cfg.err_min  = 5.0f;
  // g_cfg.k_follow = 0.30f;
  // g_cfg.load_count = 10;
}

void loop() {
  // TODO: replace with your real signals (units consistent)
  float target = 90.0f;   // desired final position (deg)
  float cmd    = 80.0f;   // commanded intermediate position (deg)
  float meas   = 75.0f;   // measured position (deg)
  float err    = (cmd - meas);  // position error (deg)

  // Example expected velocity (deg/s). Replace with your profile output.
  // If you don't have vexp, compute it from your trajectory generator, or set it to 0 when not moving.
  float vexp   = 60.0f;

  logServoLine(g_cfg, g_st, target, cmd, meas, vexp, err);

  delay(20); // just for this demo
}
