// SDM27EV BMS temperature board firmware v1.0
#include <Arduino.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include <esp_attr.h>
#include <math.h>

// Settings

// pins, checked against the PCB
const int MUX_OUT[4] = {13, 10, 11, 12}; // mux 1-4 outputs (J2, J3, J4, J5)
const int MUX_SEL[4] = {14, 21, 47, 48}; // S0-S3, shared by all 4 muxes
const int KILL_PIN = 6;                  // goes to the optocoupler, then SHUTOFF_ISO
const int FAULT_LED = 17;                // red
const int HEART_LED = 18;                // green
const int ORION_PIN = 7;                 // Orion fault output (12V divided down)
// don't use IO15 (no protection), IO35-37 (internal PSRAM) or IO38 (no resistor)

// Orion input
// Level on IO7 that means the Orion has a fault. HIGH is the fail-safe
// setup: Orion output on (pulling low) = OK, so a broken wire reads as a fault.
const int ORION_FAULT_LEVEL = HIGH;
// give the Orion time to boot after power up. During this time an Orion
// fault keeps the car off but doesn't latch.
const uint32_t ORION_STARTUP_GRACE_MS = 5000;

// Manual start
// If true, the car stays off after power up (even if everything is OK)
// until you type s and press Enter in the serial monitor. Typing x stops
// the car again without latching a fault.
// If false, the car is allowed to run as soon as all the checks pass.
// A start is refused while there's any fault. It never overrides a fault
// and never clears a latched fault, only a power cycle does that.
const bool REQUIRE_MANUAL_START = false;

// temperature limit, check the rulebook
const float OVERTEMP_C = 58.0; //2C below the 60C cell/rule limit to allow for measurement error

// divider and thermistor
const float VREF_MV = 3300.0;   // measure the 3.3V rail and update this
const float R_FIXED = 10000.0;  // 10k pull-down on each mux output
const float R25 = 10000.0;      // thermistor resistance at 25C
const float BETA = 3435.0;      // confirm from the thermistor datasheet
const float R_MUX = 0.0;        // mux on-resistance, set after calibration (around 100)

// broken sensor detection
const int OPEN_BELOW_MV = 100;   // about -45C or colder, sensor unplugged
const int SHORT_ABOVE_MV = 3050; // about 105C or hotter, sensor shorted (also overtemp)

// which channels have a thermistor fitted (1 = fitted)
// index is mux*16 + input. Input n uses connector pins 2n+1 and 2n+2.
const uint8_t CH_ENABLED[64] = {
  1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, // mux 1 (J2) inputs 0-15
  1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, // mux 2 (J3)
  1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, // mux 3 (J4)
  1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, // mux 4 (J5)
};

// timing and filtering
const uint32_t SCAN_PERIOD_MS = 50;     // 20 scans a second
const int SETTLE_US = 100;              // wait after switching the mux
const int SAMPLES = 8;                  // ADC reads averaged per channel
const int MEDIAN_N = 5;                 // readings kept per channel
const uint8_t FAULT_CONFIRM_SCANS = 5;  // scans a temp/sensor fault has to last
const uint8_t ORION_CONFIRM_SCANS = 2;  // scans an Orion fault has to last
const uint32_t WDT_TIMEOUT_MS = 1000;
const uint32_t STATUS_PERIOD_MS = 1000; // how often the status line prints

// State

enum Fault : uint8_t { F_NONE, F_OVERTEMP, F_OPEN, F_SHORT, F_ORION, F_CRASH };
const char *FAULT_NAME[] = {"none", "OVERTEMP", "SENSOR OPEN", "SENSOR SHORT",
                            "ORION FAULT", "BOARD RESET/CRASH"};

uint16_t history[64][MEDIAN_N]; // last few readings for each channel in mV (ring buffer)
uint8_t histSlot = 0;           // where the next reading goes
uint16_t filtered[64];          // median of each channel's history in mV
uint8_t badCount[64];           // bad scans in a row for each channel
uint8_t orionBadCount = 0;

int overtempMv;                 // mV reading that matches OVERTEMP_C
bool primed = false;            // history has been filled with real readings
bool orionHolding = false;      // Orion not OK yet during the startup grace time
bool started = !REQUIRE_MANUAL_START;
bool latched = false;
Fault fault = F_NONE;
int faultCh = -1;
uint32_t lastScanUs = 0;

// these survive a watchdog or crash reset but not a power cycle
const uint32_t LATCH_MAGIC = 0xB5A7E001;
RTC_NOINIT_ATTR uint32_t rtcMagic;
RTC_NOINIT_ATTR uint8_t rtcFault;
RTC_NOINIT_ATTR int8_t rtcChannel;

// Conversions

// temperature to the expected mV reading, only used once at boot for the thresholds
int celsiusToMv(float c) {
  float rTherm = R25 * expf(BETA * (1.0 / (c + 273.15) - 1.0 / 298.15));
  return (int)(VREF_MV * R_FIXED / (rTherm + R_MUX + R_FIXED));
}

// mV reading to temperature, only used for printing, not for the safety checks
float mvToCelsius(int mv) {
  if (mv <= 0) return -99.0;
  float rTherm = R_FIXED * (VREF_MV - mv) / mv - R_MUX;
  if (rTherm <= 0) return 199.0;
  return 1.0 / (1.0 / 298.15 + logf(rTherm / R25) / BETA) - 273.15;
}

// Reading

void selectInput(int addr) {
  for (int b = 0; b < 4; b++) {
    digitalWrite(MUX_SEL[b], (addr >> b) & 1);
  }
}

// reads all 64 channels into out[] in mV
void scanAll(uint16_t out[64]) {
  for (int addr = 0; addr < 16; addr++) {
    selectInput(addr);
    delayMicroseconds(SETTLE_US);
    for (int m = 0; m < 4; m++) {
      analogReadMilliVolts(MUX_OUT[m]); // throw away the first reading
      uint32_t sum = 0;
      for (int s = 0; s < SAMPLES; s++) {
        sum += analogReadMilliVolts(MUX_OUT[m]);
      }
      out[m * 16 + addr] = sum / SAMPLES; // channel = mux*16 + input
    }
  }
}

// middle value of a channel's history, sorts a copy
uint16_t median(const uint16_t *v) {
  uint16_t s[MEDIAN_N];
  memcpy(s, v, sizeof(s));
  for (int i = 1; i < MEDIAN_N; i++) {
    uint16_t x = s[i];
    int j = i - 1;
    while (j >= 0 && s[j] > x) {
      s[j + 1] = s[j];
      j--;
    }
    s[j + 1] = x;
  }
  return s[MEDIAN_N / 2];
}

// Fault logic

Fault classify(uint16_t mv) {
  if (mv < OPEN_BELOW_MV) return F_OPEN;
  if (mv > SHORT_ABOVE_MV) return F_SHORT;
  if (mv >= overtempMv) return F_OVERTEMP;
  return F_NONE;
}

void printChannel(int ch) {
  int m = ch / 16;
  int in = ch % 16;
  Serial.printf("ch %d (mux %d input %d, J%d pins %d-%d)",
                ch, m + 1, in, m + 2, 2 * in + 1, 2 * in + 2);
}

void latchFault(Fault f, int ch) {
  if (latched) return; // keep the first fault
  latched = true;
  fault = f;
  faultCh = ch;
  rtcMagic = LATCH_MAGIC;
  rtcFault = f;
  rtcChannel = ch;
  digitalWrite(KILL_PIN, LOW); // kill right away
  Serial.printf("\nFAULT LATCHED: %s", FAULT_NAME[f]);
  if (ch >= 0) {
    Serial.print(" on ");
    printChannel(ch);
  }
  Serial.println();
}

void evaluate(uint32_t now) {
  // temperatures and sensors
  for (int ch = 0; ch < 64; ch++) {
    if (!CH_ENABLED[ch]) continue;
    filtered[ch] = median(history[ch]);
    Fault f = classify(filtered[ch]);
    if (f == F_NONE) {
      badCount[ch] = 0;
      continue;
    }
    if (badCount[ch] < 255) badCount[ch]++;
    if (badCount[ch] >= FAULT_CONFIRM_SCANS) latchFault(f, ch);
  }

  // Orion
  bool orionBad = digitalRead(ORION_PIN) == ORION_FAULT_LEVEL;
  if (!orionBad) orionBadCount = 0;
  else if (orionBadCount < 255) orionBadCount++;

  orionHolding = false;
  if (orionBadCount >= ORION_CONFIRM_SCANS) {
    if (now < ORION_STARTUP_GRACE_MS) orionHolding = true; // Orion still booting
    else latchFault(F_ORION, -1);
  }
}

// true if anything looks wrong right now, even if it hasn't latched yet
bool anyProblemNow() {
  if (latched || orionHolding || !primed) return true;
  if (digitalRead(ORION_PIN) == ORION_FAULT_LEVEL) return true;
  for (int ch = 0; ch < 64; ch++) {
    if (CH_ENABLED[ch] && classify(filtered[ch]) != F_NONE) return true;
  }
  return false;
}

void tryStart() {
  if (started) {
    Serial.println("Already started");
  } else if (latched) {
    Serial.println("Start refused: fault latched, power cycle the board to clear it");
  } else if (anyProblemNow()) {
    Serial.println("Start refused: something isn't OK yet, check the status line");
  } else {
    started = true;
    Serial.println("Car started");
  }
}

void updateOutputs(uint32_t now) {
  bool carOk = primed && started && !latched && !orionHolding;
  digitalWrite(KILL_PIN, carOk ? HIGH : LOW);

  // red: solid for a temp/sensor/crash fault, blinking for an Orion fault
  bool blink = (now / 250) % 2;
  if (!latched) digitalWrite(FAULT_LED, orionHolding ? blink : LOW);
  else if (fault == F_ORION) digitalWrite(FAULT_LED, blink);
  else digitalWrite(FAULT_LED, HIGH);

  // green: slow blink when running, fast blink when waiting for a manual start
  if (started) digitalWrite(HEART_LED, (now / 500) % 2);
  else digitalWrite(HEART_LED, (now / 125) % 2);
}

// USB output

void printStatus() {
  if (latched) {
    Serial.printf("KILL: %s", FAULT_NAME[fault]);
    if (faultCh >= 0) {
      Serial.print(" on ");
      printChannel(faultCh);
    }
    Serial.print(" | ");
  } else if (orionHolding) {
    Serial.print("KILL: waiting for Orion OK | ");
  } else if (!started) {
    Serial.print("OFF: waiting for start (type s) | ");
  } else {
    Serial.print("OK | ");
  }

  int lo = 9999, hi = -1, hiCh = -1, n = 0;
  long sum = 0;
  for (int ch = 0; ch < 64; ch++) {
    if (!CH_ENABLED[ch]) continue;
    Fault f = classify(filtered[ch]);
    if (f == F_OPEN || f == F_SHORT) continue;
    int v = filtered[ch];
    if (v < lo) lo = v;
    if (v > hi) {
      hi = v;
      hiCh = ch;
    }
    sum += v;
    n++;
  }
  if (n > 0) {
    Serial.printf("min %.1fC max %.1fC (ch %d) avg %.1fC | ",
                  mvToCelsius(lo), mvToCelsius(hi), hiCh, mvToCelsius(sum / n));
  } else {
    Serial.print("no working sensors | ");
  }
  Serial.printf("Orion %s | scan %.1f ms\n",
                digitalRead(ORION_PIN) == ORION_FAULT_LEVEL ? "FAULT" : "ok",
                lastScanUs / 1000.0);
}

void printTable() {
  Serial.println("\nInput | Mux 1 (J2) | Mux 2 (J3) | Mux 3 (J4) | Mux 4 (J5)");
  for (int in = 0; in < 16; in++) {
    Serial.printf("%2d", in);
    for (int m = 0; m < 4; m++) {
      int ch = m * 16 + in;
      int v = filtered[ch];
      if (!CH_ENABLED[ch]) Serial.print(" | off");
      else if (v < OPEN_BELOW_MV) Serial.printf(" | %dmV OPEN", v);
      else if (v > SHORT_ABOVE_MV) Serial.printf(" | %dmV SHORT", v);
      else Serial.printf(" | %dmV %.1fC", v, mvToCelsius(v));
    }
    Serial.println();
  }
}

// t = print table, s = start, x = stop
void handleCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 't') printTable();
    else if (c == 's') tryStart();
    else if (c == 'x') {
      if (started) Serial.println("Car stopped");
      started = false;
    }
  }
}

void setup() {
  // set the kill output first, before anything else
  pinMode(KILL_PIN, OUTPUT);
  digitalWrite(KILL_PIN, LOW);
  pinMode(FAULT_LED, OUTPUT);
  digitalWrite(FAULT_LED, HIGH);
  pinMode(HEART_LED, OUTPUT);

  Serial.setTxBufferSize(1024);
  Serial.begin(115200);

  // inputs and ADC
  pinMode(ORION_PIN, INPUT); // the board already has a pull-up/divider on this
  for (int b = 0; b < 4; b++) pinMode(MUX_SEL[b], OUTPUT);
  analogReadResolution(12);
  for (int m = 0; m < 4; m++) analogSetPinAttenuation(MUX_OUT[m], ADC_11db);
  overtempMv = celsiusToMv(OVERTEMP_C);

  // keep or clear the latch depending on why the board reset.
  // a power cycle counts as the manual reset.
  esp_reset_reason_t why = esp_reset_reason();
  if (why == ESP_RST_POWERON) rtcMagic = 0;
  if (rtcMagic == LATCH_MAGIC) {
    latched = true;
    fault = (Fault)rtcFault;
    faultCh = rtcChannel;
  }
  bool crashed = why == ESP_RST_TASK_WDT || why == ESP_RST_INT_WDT ||
                 why == ESP_RST_WDT || why == ESP_RST_PANIC;

  delay(200);
  Serial.println("\nSDM27EV BMS temp board v0.1");
  Serial.printf("Reset reason %d, overtemp %.1fC = %d mV, open below %d mV, short above %d mV\n",
                why, OVERTEMP_C, overtempMv, OPEN_BELOW_MV, SHORT_ABOVE_MV);
  if (latched) Serial.println("Fault latch restored from before the reset");
  if (crashed) latchFault(F_CRASH, -1);

  // watchdog, resets the board if loop() stops running
  esp_task_wdt_config_t wdt;
  wdt.timeout_ms = WDT_TIMEOUT_MS;
  wdt.idle_core_mask = 0;
  wdt.trigger_panic = true;
  if (esp_task_wdt_reconfigure(&wdt) != ESP_OK) esp_task_wdt_init(&wdt);
  esp_task_wdt_add(NULL);

  // fill every channel's history before trusting any reading
  for (int i = 0; i < MEDIAN_N; i++) {
    uint16_t raw[64];
    scanAll(raw);
    for (int ch = 0; ch < 64; ch++) history[ch][i] = raw[ch];
    esp_task_wdt_reset();
  }
  primed = true;
  Serial.println("Ready. Commands: t = table, s = start, x = stop");
}

void loop() {
  static uint32_t nextScan = 0;
  static uint32_t nextStatus = 0;
  uint32_t now = millis();

  if ((int32_t)(now - nextScan) >= 0) {
    nextScan = now + SCAN_PERIOD_MS;
    uint32_t t0 = micros();

    uint16_t raw[64];
    scanAll(raw);
    for (int ch = 0; ch < 64; ch++) history[ch][histSlot] = raw[ch];
    histSlot = (histSlot + 1) % MEDIAN_N; // wrap back to the start

    evaluate(now);
    updateOutputs(now);

    lastScanUs = micros() - t0;
    esp_task_wdt_reset(); // tell the watchdog we're still running
  }

  if ((int32_t)(now - nextStatus) >= 0) {
    nextStatus = now + STATUS_PERIOD_MS;
    printStatus();
  }

  handleCommands();
}
