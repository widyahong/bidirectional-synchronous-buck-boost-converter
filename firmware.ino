// Open-loop bidirectional synchronous buck-boost (4-switch), PORT_A <-> PORT_B
// ESP32, Arduino IDE, no external libraries (ESP-IDF LEDC driver ships with the core).
// No sensing, no regulation. One duty D drives both legs together:
//   Q1(HIN_A) + Q4(LIN_B) ON for D, then Q2(LIN_A) + Q3(HIN_B) ON for 1-D, with dead-time.
// Balance (zero-current) duty D0 = VB / (VA + VB). D = D0 +/- offset by direction.
// Serial 115200: type "help".

#include <Arduino.h>
#include "driver/ledc.h"

// ---------------- Pins / LEDC (all channels on ONE timer) ----------------
const uint8_t PIN_HIN_A = 25, PIN_LIN_A = 26, PIN_HIN_B = 32, PIN_LIN_B = 33;
const ledc_mode_t    MODE = LEDC_LOW_SPEED_MODE;
const ledc_timer_t   TMR  = LEDC_TIMER_0;
const ledc_channel_t CH_HIN_A = LEDC_CHANNEL_0, CH_LIN_A = LEDC_CHANNEL_1,
                     CH_HIN_B = LEDC_CHANNEL_2, CH_LIN_B = LEDC_CHANNEL_3;

// ---------------- User variables (also settable via Serial) ----------------
uint32_t freqHz    = 50000;   // 20000..200000 (stopped only)
uint16_t deadNs    = 300;     // dead-time in ns (stopped only)
float    vA        = 12.0f;   // nominal port volts, ONLY used to compute D0 (stopped only)
float    vB        = 12.0f;
int8_t   direction = 1;       // +1: PORT_A -> PORT_B, -1: PORT_B -> PORT_A
float    offsetPct = 0.0f;    // duty points away from D0 (current strength, unregulated)
float    rampStep  = 0.1f;    // % duty per ramp tick
uint16_t rampMs    = 50;      // ramp tick interval

const float    DUTY_MIN       = 5.0f;   // bootstrap needs low-side time
const float    DUTY_MAX       = 90.0f;
const float    OFFSET_MAX     = 5.0f;   // raise in code only if you know what you are doing
const uint16_t BOOT_CHARGE_MS = 5;
const uint32_t TIMER_CLK_HZ   = 80000000UL;  // APB assumption, used to pick resolution

// ---------------- State ----------------
bool     running = false;
uint8_t  resBits = 10;
float    curD    = 50.0f;
uint32_t lastRamp = 0;
String   rxBuf;

// ---------------- Helpers ----------------
uint32_t periodCounts() { return 1UL << resBits; }

uint32_t deadCounts() {
  float c = (float)deadNs * 1e-9f * (float)freqHz * (float)periodCounts() + 0.5f;
  uint32_t n = (uint32_t)c;
  return n < 2 ? 2 : n;
}

void calcResolution() {
  resBits = 1;
  while (resBits < 14 && (TIMER_CLK_HZ / freqHz) >= (1UL << (resBits + 1))) resBits++;
}

float balanceDuty() { return 100.0f * vB / (vA + vB); }

float targetDuty() {
  float d = balanceDuty() + (float)direction * offsetPct;
  return constrain(d, DUTY_MIN, DUTY_MAX);
}

void writeCh(ledc_channel_t ch, uint32_t duty, uint32_t hpoint) {
  ledc_set_duty_with_hpoint(MODE, ch, duty, hpoint);
  ledc_update_duty(MODE, ch);
}

// H pattern [0, h)        -> Q1 (HIN_A) and Q4 (LIN_B)
// L pattern [h+dt, P-dt)  -> Q2 (LIN_A) and Q3 (HIN_B)
// Update order: L-pattern/idle-high channels first, so the first cycle after start can
// never have a high side ON while its low side is still held ON.
void applyDuty(float d) {
  uint32_t P = periodCounts(), dt = deadCounts();
  uint32_t h = (uint32_t)(d * 0.01f * (float)P + 0.5f);
  if (h + 2 * dt + 2 >= P) h = P - 2 * dt - 2;
  uint32_t l = P - h - 2 * dt;
  writeCh(CH_LIN_A, l, h + dt);
  writeCh(CH_LIN_B, h, 0);
  writeCh(CH_HIN_A, h, 0);
  writeCh(CH_HIN_B, l, h + dt);
}

void allLow() {   // only before LEDC is configured (setup)
  const uint8_t pins[4] = {PIN_HIN_A, PIN_LIN_A, PIN_HIN_B, PIN_LIN_B};
  for (uint8_t i = 0; i < 4; i++) { pinMode(pins[i], OUTPUT); digitalWrite(pins[i], LOW); }
}

bool initChannel(uint8_t pin, ledc_channel_t ch) {
  ledc_channel_config_t c = {};
  c.gpio_num   = pin;
  c.speed_mode = MODE;
  c.channel    = ch;
  c.intr_type  = LEDC_INTR_DISABLE;
  c.timer_sel  = TMR;
  c.duty       = 0;
  c.hpoint     = 0;
  return ledc_channel_config(&c) == ESP_OK;
}

void stopPwm() {
  if (running) {
    ledc_stop(MODE, CH_HIN_A, 0); ledc_stop(MODE, CH_LIN_A, 0);
    ledc_stop(MODE, CH_HIN_B, 0); ledc_stop(MODE, CH_LIN_B, 0);
  }
  running = false;
}

void startPwm() {
  calcResolution();
  uint32_t P = periodCounts(), dt = deadCounts();
  float d0 = balanceDuty();
  if (dt * 20 >= P) { Serial.println("ERR: dead-time too long for this frequency"); return; }
  if (d0 < DUTY_MIN || d0 > DUTY_MAX) { Serial.println("ERR: D0 outside 5..90 %, check va/vb"); return; }

  ledc_timer_config_t t = {};
  t.speed_mode      = MODE;
  t.duty_resolution = (ledc_timer_bit_t)resBits;
  t.timer_num       = TMR;
  t.freq_hz         = freqHz;
  t.clk_cfg         = LEDC_AUTO_CLK;
  if (ledc_timer_config(&t) != ESP_OK) { Serial.println("ERR: LEDC timer config failed"); return; }

  if (!initChannel(PIN_HIN_A, CH_HIN_A) || !initChannel(PIN_LIN_A, CH_LIN_A) ||
      !initChannel(PIN_HIN_B, CH_HIN_B) || !initChannel(PIN_LIN_B, CH_LIN_B)) {
    Serial.println("ERR: LEDC channel config failed"); return;
  }

  // Bootstrap charge: both low sides ON, high sides OFF (inductor shorted to GND, no current)
  ledc_stop(MODE, CH_LIN_A, 1);
  ledc_stop(MODE, CH_LIN_B, 1);
  delay(BOOT_CHARGE_MS);

  curD = d0;                 // start at balance = zero average current
  applyDuty(curD);
  lastRamp = millis();
  running = true;
}

float stepToward(float cur, float target, float s) {
  if (cur < target) return min(cur + s, target);
  if (cur > target) return max(cur - s, target);
  return cur;
}

void rampTick() {
  // Step is limited to < dead-time so a live update can never overlap H and L of one leg
  float limit = 50.0f * (float)deadCounts() / (float)periodCounts();
  float s = min(rampStep, limit);
  float n = stepToward(curD, targetDuty(), s);
  if (n != curD) { curD = n; applyDuty(curD); }
}

void printStatus() {
  Serial.printf("running=%d freq=%lu Hz res=%u bit (P=%lu, 1 step=%.3f%%) dead=%u ns (%lu cnt)\n",
                running, (unsigned long)freqHz, resBits, (unsigned long)periodCounts(),
                100.0f / (float)periodCounts(), deadNs, (unsigned long)deadCounts());
  Serial.printf("vA=%.2f V vB=%.2f V -> D0=%.2f%% | dir=%s offset=%.2f%% -> target D=%.2f%% now D=%.2f%%\n",
                vA, vB, balanceDuty(), direction > 0 ? "A2B" : "B2A",
                offsetPct, targetDuty(), curD);
  Serial.printf("ramp step=%.2f%% every %u ms\n", rampStep, rampMs);
}

void printHelp() {
  Serial.println("start | stop | status | help");
  Serial.println("dir a2b|b2a      energy flow direction (live)");
  Serial.println("off <%>          duty points away from balance D0, 0..5 (live)");
  Serial.println("step <%>  rate <ms>   ramp settings (live)");
  Serial.println("va <V>  vb <V>  freq <Hz>  dead <ns>   (only while stopped)");
  Serial.println("Open loop: tenths of a percent of offset can already mean amps.");
}

void handleLine(String s) {
  s.trim(); s.toLowerCase();
  if (!s.length()) return;
  int sp = s.indexOf(' ');
  String cmd = (sp < 0) ? s : s.substring(0, sp);
  String arg = (sp < 0) ? "" : s.substring(sp + 1);
  arg.trim();
  float v = arg.toFloat();

  bool needStop = (cmd == "va" || cmd == "vb" || cmd == "freq" || cmd == "dead");
  if (needStop && running) { Serial.println("ERR: stop first"); return; }

  if      (cmd == "start") { if (!running) startPwm(); }
  else if (cmd == "stop")  { stopPwm(); }
  else if (cmd == "dir")   {
    if (arg == "a2b") direction = 1;
    else if (arg == "b2a") direction = -1;
    else { Serial.println("ERR: dir a2b|b2a"); return; }
  }
  else if (cmd == "off")   { offsetPct = constrain(v, 0.0f, OFFSET_MAX); }
  else if (cmd == "step")  { rampStep = constrain(v, 0.05f, 5.0f); }
  else if (cmd == "rate")  { rampMs = (uint16_t)constrain(v, 1.0f, 1000.0f); }
  else if (cmd == "va")    { vA = constrain(v, 1.0f, 100.0f); }
  else if (cmd == "vb")    { vB = constrain(v, 1.0f, 100.0f); }
  else if (cmd == "freq")  { freqHz = (uint32_t)constrain(v, 20000.0f, 200000.0f); }
  else if (cmd == "dead")  { deadNs = (uint16_t)constrain(v, 50.0f, 2000.0f); }
  else if (cmd == "help")  { printHelp(); return; }
  else if (cmd != "status") { Serial.println("ERR: unknown command"); return; }
  printStatus();
}

void setup() {
  allLow();                  // all gates OFF first (external 10k pull-downs cover boot)
  Serial.begin(115200);
  delay(300);
  Serial.println("Open-loop bidirectional buck-boost, PORT_A <-> PORT_B (no regulation, no sensing)");
  printHelp();
  printStatus();
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') { handleLine(rxBuf); rxBuf = ""; }
    else if (rxBuf.length() < 40) rxBuf += c;
  }
  if (running && (millis() - lastRamp) >= rampMs) {
    lastRamp = millis();
    rampTick();
  }
}
