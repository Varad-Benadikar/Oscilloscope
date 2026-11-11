#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(128, 64, &Wire, -1);

// ===== ADC PINS =====
#define ADC_PIN_A   34
#define ADC_PIN_B   35

// ===== BUTTONS =====
#define AMP_UP_A    25
#define AMP_DOWN_A  26
#define TIME_UP_A   27
#define TIME_DOWN_A 14
#define AMP_UP_B    5
#define AMP_DOWN_B  18
#define TIME_UP_B   12
#define TIME_DOWN_B 13
#define CH_A_BTN    23
#define CH_B_BTN    4
#define HOLD_BTN    33
#define MODE_BTN    32

// ===== CIRCUIT CONSTANTS =====
#define TRUE_ATTENUATION  13.0
#define VREF              3.3
#define ADC_MAX           4095.0
const int FIXED_MID     = 2048;

// ===== HYSTERESIS =====
#define HYSTERESIS_BAND   100

// ===== SMOOTHING =====
#define SMOOTH_FACTOR     0.1
#define STATS_UPDATE_MS   300

// ===== DEBOUNCE =====
#define DEBOUNCE_MS       120

// ===== AUTO CALIBRATION =====
#define CALIB_SAMPLES     200
int adcOffsetA = 0;
int adcOffsetB = 0;

// ===== CHANNEL FLAGS =====
bool chA_active = true;
bool chB_active = false;

// ===== AMP / TIME SCALES =====
float ampScaleA = 3.0;
float ampScaleB = 3.0;
int   timeScaleA = 1;
int   timeScaleB = 1;

// ===== SAMPLES =====
uint16_t samplesA[128];
uint16_t samplesB[128];

// ===== HOLD / MODE =====
bool holdWave = false;
// menuMode: 0=waveform, 1=stats A, 2=stats B, 3=hyst stats A, 4=hyst stats B / split
int  menuMode = 0;

// ===== STATS RAW =====
float vppA, vmaxA, vminA, vdcA, freqHzA;
float vppB, vmaxB, vminB, vdcB, freqHzB;
float freqHzA_hyst = 0;
float freqHzB_hyst = 0;

// ===== STATS SMOOTHED =====
float dispVppA=0, dispVmaxA=0, dispVminA=0, dispVdcA=0, dispFreqA=0;
float dispVppB=0, dispVmaxB=0, dispVminB=0, dispVdcB=0, dispFreqB=0;
float dispFreqA_hyst=0, dispFreqB_hyst=0;
unsigned long lastStatsUpdate = 0;

// ===== FREQUENCY AVERAGING =====
#define FREQ_AVG_COUNT 10
float freqHistoryA[FREQ_AVG_COUNT];
int   freqIndexA = 0;
bool  freqFilledA = false;
float freqHistoryB[FREQ_AVG_COUNT];
int   freqIndexB = 0;
bool  freqFilledB = false;

float freqHistoryA_hyst[FREQ_AVG_COUNT];
int   freqIndexA_hyst = 0;
bool  freqFilledA_hyst = false;
float freqHistoryB_hyst[FREQ_AVG_COUNT];
int   freqIndexB_hyst = 0;
bool  freqFilledB_hyst = false;

// ===== CAPTURE TIME =====
float captureTimeA = 0.0;
float captureTimeB = 0.0;

// ===== PRESS DEBOUNCE =====
unsigned long lastAmpUpA   = 0, lastAmpDownA  = 0;
unsigned long lastTimeUpA  = 0, lastTimeDownA = 0;
unsigned long lastAmpUpB   = 0, lastAmpDownB  = 0;
unsigned long lastTimeUpB  = 0, lastTimeDownB = 0;

// ===== RELEASE DETECTION =====
bool prevChA  = HIGH, prevChB  = HIGH;
bool prevHold = HIGH, prevMode = HIGH;

bool buttonPressed(int pin, unsigned long &lastTime) {
  if (digitalRead(pin) == LOW) {
    unsigned long now = millis();
    if (now - lastTime >= DEBOUNCE_MS) {
      lastTime = now;
      return true;
    }
  }
  return false;
}

bool buttonReleased(int pin, bool &prevState) {
  bool current = digitalRead(pin);
  if (prevState == LOW && current == HIGH) {
    prevState = HIGH;
    return true;
  }
  prevState = current;
  return false;
}

void clearSamples(uint16_t* samples) {
  for (int i = 0; i < 128; i++) samples[i] = (uint16_t)FIXED_MID;
}

void showSplashScreen() {
  display.clearDisplay();
  display.setTextSize(4);
  display.setTextColor(WHITE);
  display.setCursor(4, 10);
  display.print("Varad");
  display.setTextSize(1);
  display.setCursor(22, 52);
  display.print("Oscilloscope v1.0");
  display.display();
  delay(3000);
}

void autoCalibrate() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(10, 25);
  display.print("Calibrating...");
  display.display();

  long sumA = 0, sumB = 0;
  for (int i = 0; i < CALIB_SAMPLES; i++) {
    sumA += analogRead(ADC_PIN_A);
    sumB += analogRead(ADC_PIN_B);
    delay(1);
  }

  int avgA = (int)(sumA / CALIB_SAMPLES);
  int avgB = (int)(sumB / CALIB_SAMPLES);

  adcOffsetA = FIXED_MID - avgA;
  adcOffsetB = FIXED_MID - avgB;

  display.clearDisplay();
  display.setCursor(10, 15);
  display.print("Cal done.");
  display.setCursor(0, 28);
  display.print("A offset:");
  display.print(adcOffsetA);
  display.setCursor(0, 40);
  display.print("B offset:");
  display.print(adcOffsetB);
  display.display();
  delay(1500);
}

float adcToActualVolts(int rawAdcValue) {
  float centeredAdc     = rawAdcValue - FIXED_MID;
  float pinVoltageSwing = (centeredAdc / (ADC_MAX / 2.0)) * (VREF / 2.0);
  return pinVoltageSwing * TRUE_ATTENUATION;
}

void calcStats(uint16_t* samples, int offset,
               float &vpp, float &vmax, float &vmin, float &vdc) {
  int rawMax = (int)samples[0] + offset;
  int rawMin = (int)samples[0] + offset;
  long sum   = 0;

  for (int i = 0; i < 128; i++) {
    int corrected = (int)samples[i] + offset;
    if (corrected > rawMax) rawMax = corrected;
    if (corrected < rawMin) rawMin = corrected;
    sum += corrected;
  }

  vmax = adcToActualVolts(rawMax);
  vmin = adcToActualVolts(rawMin);
  vpp  = vmax - vmin;
  vdc  = adcToActualVolts((int)(sum / 128));
}

void calcFreq(uint16_t* samples, float captureTime,
              float &freqHz, float* freqHistory,
              int &freqIndex, bool &freqFilled,
              bool useHysteresis) {

  long sum = 0;
  for (int i = 0; i < 128; i++) sum += samples[i];
  int waveAverage = (int)(sum / 128);

  int crossings = 0;

  if (useHysteresis) {
    int upperThresh = waveAverage + HYSTERESIS_BAND;
    int lowerThresh = waveAverage - HYSTERESIS_BAND;
    bool state = ((int)samples[0] >= upperThresh);

    for (int i = 1; i < 128; i++) {
      if (!state && (int)samples[i] >= upperThresh) {
        state = true;
        crossings++;
      } else if (state && (int)samples[i] <= lowerThresh) {
        state = false;
      }
    }
  } else {
    for (int i = 1; i < 128; i++) {
      if ((int)samples[i-1] < waveAverage &&
          (int)samples[i]   >= waveAverage) {
        crossings++;
      }
    }
  }

  float rawFreq = (crossings > 0 && captureTime > 0)
                  ? (crossings / captureTime) : 0;

  freqHistory[freqIndex] = rawFreq;
  freqIndex = (freqIndex + 1) % FREQ_AVG_COUNT;
  if (freqIndex == 0) freqFilled = true;

  int count = freqFilled ? FREQ_AVG_COUNT : freqIndex;
  float total = 0;
  for (int i = 0; i < count; i++) total += freqHistory[i];
  freqHz = (count > 0) ? (total / count) : 0;
}

void updateDisplayStats() {
  unsigned long now = millis();
  if (now - lastStatsUpdate >= STATS_UPDATE_MS) {
    lastStatsUpdate = now;

    if (chA_active) {
      dispVppA       += SMOOTH_FACTOR * (vppA         - dispVppA);
      dispVmaxA      += SMOOTH_FACTOR * (vmaxA        - dispVmaxA);
      dispVminA      += SMOOTH_FACTOR * (vminA        - dispVminA);
      dispVdcA       += SMOOTH_FACTOR * (vdcA         - dispVdcA);
      dispFreqA      += SMOOTH_FACTOR * (freqHzA      - dispFreqA);
      dispFreqA_hyst += SMOOTH_FACTOR * (freqHzA_hyst - dispFreqA_hyst);
    }

    if (chB_active) {
      dispVppB       += SMOOTH_FACTOR * (vppB         - dispVppB);
      dispVmaxB      += SMOOTH_FACTOR * (vmaxB        - dispVmaxB);
      dispVminB      += SMOOTH_FACTOR * (vminB        - dispVminB);
      dispVdcB       += SMOOTH_FACTOR * (vdcB         - dispVdcB);
      dispFreqB      += SMOOTH_FACTOR * (freqHzB      - dispFreqB);
      dispFreqB_hyst += SMOOTH_FACTOR * (freqHzB_hyst - dispFreqB_hyst);
    }
  }
}

void drawWaveform(uint16_t* samples, int offset,
                  float ampScale, bool dotted) {
  int prevY = 32;

  for (int x = 0; x < 128; x++) {
    float v = ((int)samples[x] + offset) - FIXED_MID;
    v *= ampScale;
    int y = 32 - (int)((v / (ADC_MAX / 2.0)) * 26);
    y = constrain(y, 0, 63);

    if (x > 0) {
      if (dotted) {
        if (x % 2 == 0) display.drawPixel(x, y, WHITE);
      } else {
        display.drawLine(x - 1, prevY, x, y, WHITE);
      }
    }
    prevY = y;
  }
}

void drawStats(char ch, float vpp, float vmax,
               float vmin, float vdc, float freq, bool isHyst) {
  display.setTextSize(1);
  display.setTextColor(WHITE);

  display.setCursor(0, 0);
  display.print("-- CH ");
  display.print(ch);
  if (isHyst) display.print(" [H] --");
  else        display.print(" --");

  display.setCursor(0, 12);
  display.print("Vpp: ");
  display.print(vpp, 2);
  display.print("V");

  display.setCursor(0, 22);
  display.print("Vmax:");
  display.print(vmax, 2);
  display.print("V");

  display.setCursor(0, 32);
  display.print("Vmin:");
  display.print(vmin, 2);
  display.print("V");

  display.setCursor(0, 42);
  display.print("DC:  ");
  display.print(vdc, 2);
  display.print("V");

  display.setCursor(0, 52);
  display.print("Freq:");
  if (freq >= 1000) {
    display.print(freq / 1000.0, 2);
    display.print("kHz");
  } else {
    display.print((int)freq);
    display.print("Hz");
  }
}

void drawStatusBar() {
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);

  display.print("Ch:");
  if (chA_active && chB_active)       display.print("AB");
  else if (chA_active)                display.print("A ");
  else if (chB_active)                display.print("B ");
  else                                display.print("--");

  display.print(" A:");
  if (chA_active && !chB_active)      display.print(ampScaleA, 1);
  else if (chB_active && !chA_active) display.print(ampScaleB, 1);
  else if (chA_active && chB_active)  display.print(ampScaleA, 1);
  else                                display.print("-");

  display.print(" T:");
  if (chA_active && !chB_active)      display.print(timeScaleA);
  else if (chB_active && !chA_active) display.print(timeScaleB);
  else if (chA_active && chB_active)  display.print(timeScaleA);
  else                                display.print("-");

  if (holdWave) {
    display.setCursor(104, 0);
    display.print("HLD");
  }
}

// ==============================
// Menu cycling
// Fixed sequence regardless of active channels:
// 0(wave) → 1(CH A stats) → 2(CH B stats) → 3(hyst waveform) → 4(hyst values) → 0
// Inactive channel pages show "CH X inactive" message
// ==============================
void handleMenuPress() {
  menuMode = (menuMode + 1) % 5;
}

// ==============================
// Draw hysteresis waveform view
// Same as normal waveform but label shows [H] and uses hyst freq in status
// ==============================
void drawHystStatusBar() {
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);

  display.print("Ch:");
  if (chA_active && chB_active)       display.print("AB");
  else if (chA_active)                display.print("A ");
  else if (chB_active)                display.print("B ");
  else                                display.print("--");

  display.print(" [H]");

  if (holdWave) {
    display.setCursor(104, 0);
    display.print("HLD");
  }
}

// ==============================
// SETUP
// ==============================
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  pinMode(AMP_UP_A,    INPUT_PULLUP);
  pinMode(AMP_DOWN_A,  INPUT_PULLUP);
  pinMode(TIME_UP_A,   INPUT_PULLUP);
  pinMode(TIME_DOWN_A, INPUT_PULLUP);
  pinMode(AMP_UP_B,    INPUT_PULLUP);
  pinMode(AMP_DOWN_B,  INPUT_PULLUP);
  pinMode(TIME_UP_B,   INPUT_PULLUP);
  pinMode(TIME_DOWN_B, INPUT_PULLUP);
  pinMode(CH_A_BTN,    INPUT_PULLUP);
  pinMode(CH_B_BTN,    INPUT_PULLUP);
  pinMode(HOLD_BTN,    INPUT_PULLUP);
  pinMode(MODE_BTN,    INPUT_PULLUP);

  for (int i = 0; i < FREQ_AVG_COUNT; i++) {
    freqHistoryA[i]      = 0;
    freqHistoryB[i]      = 0;
    freqHistoryA_hyst[i] = 0;
    freqHistoryB_hyst[i] = 0;
  }

  clearSamples(samplesA);
  clearSamples(samplesB);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.display();

  showSplashScreen();
  autoCalibrate();
}

// ==============================
// LOOP
// ==============================
void loop() {

  // ===== TOGGLE BUTTONS =====

  if (buttonReleased(CH_A_BTN, prevChA)) {
    bool wasA = chA_active;
    chA_active = !chA_active;
    if (!wasA && chA_active) clearSamples(samplesA);
    menuMode = 0;
  }

  if (buttonReleased(CH_B_BTN, prevChB)) {
    bool wasB = chB_active;
    chB_active = !chB_active;
    if (!wasB && chB_active) clearSamples(samplesB);
    menuMode = 0;
  }

  if (buttonReleased(HOLD_BTN, prevHold)) {
    holdWave = !holdWave;
  }

  if (buttonReleased(MODE_BTN, prevMode)) {
    handleMenuPress();
  }

  // ===== REPEAT BUTTONS =====

  if (chA_active) {
    if (buttonPressed(AMP_UP_A,    lastAmpUpA))   { ampScaleA  += 0.2; if (ampScaleA  > 15)  ampScaleA  = 15;  }
    if (buttonPressed(AMP_DOWN_A,  lastAmpDownA)) { ampScaleA  -= 0.2; if (ampScaleA  < 0.2) ampScaleA  = 0.2; }
    if (buttonPressed(TIME_UP_A,   lastTimeUpA))  { timeScaleA++;      if (timeScaleA > 8)   timeScaleA = 8;   }
    if (buttonPressed(TIME_DOWN_A, lastTimeDownA)){ timeScaleA--;      if (timeScaleA < 1)   timeScaleA = 1;   }
  }

  if (chB_active) {
    if (buttonPressed(AMP_UP_B,    lastAmpUpB))   { ampScaleB  += 0.2; if (ampScaleB  > 15)  ampScaleB  = 15;  }
    if (buttonPressed(AMP_DOWN_B,  lastAmpDownB)) { ampScaleB  -= 0.2; if (ampScaleB  < 0.2) ampScaleB  = 0.2; }
    if (buttonPressed(TIME_UP_B,   lastTimeUpB))  { timeScaleB++;      if (timeScaleB > 8)   timeScaleB = 8;   }
    if (buttonPressed(TIME_DOWN_B, lastTimeDownB)){ timeScaleB--;      if (timeScaleB < 1)   timeScaleB = 1;   }
  }

  // ===== ACQUIRE SAMPLES =====

  bool bothActive = chA_active && chB_active;

  if (!holdWave) {
    if (bothActive) {
      unsigned long startTime = micros();
      for (int i = 0; i < 128; i++) {
        for (int s = 0; s < timeScaleA - 1; s++) analogRead(ADC_PIN_A);
        for (int s = 0; s < timeScaleB - 1; s++) analogRead(ADC_PIN_B);
        samplesA[i] = (uint16_t)analogRead(ADC_PIN_A);
        samplesB[i] = (uint16_t)analogRead(ADC_PIN_B);
      }
      unsigned long endTime = micros();
      captureTimeA = (endTime - startTime) / 1000000.0;
      captureTimeB = captureTimeA;

    } else {
      if (chA_active) {
        unsigned long startTime = micros();
        for (int i = 0; i < 128; i++) {
          for (int s = 0; s < timeScaleA - 1; s++) analogRead(ADC_PIN_A);
          samplesA[i] = (uint16_t)analogRead(ADC_PIN_A);
        }
        captureTimeA = (micros() - startTime) / 1000000.0;
      }

      if (chB_active) {
        unsigned long startTime = micros();
        for (int i = 0; i < 128; i++) {
          for (int s = 0; s < timeScaleB - 1; s++) analogRead(ADC_PIN_B);
          samplesB[i] = (uint16_t)analogRead(ADC_PIN_B);
        }
        captureTimeB = (micros() - startTime) / 1000000.0;
      }
    }
  }

  // ===== CALCULATE =====

  if (chA_active) {
    calcStats(samplesA, adcOffsetA, vppA, vmaxA, vminA, vdcA);
    calcFreq(samplesA, captureTimeA, freqHzA,
             freqHistoryA, freqIndexA, freqFilledA, false);
    calcFreq(samplesA, captureTimeA, freqHzA_hyst,
             freqHistoryA_hyst, freqIndexA_hyst, freqFilledA_hyst, true);
  }

  if (chB_active) {
    calcStats(samplesB, adcOffsetB, vppB, vmaxB, vminB, vdcB);
    calcFreq(samplesB, captureTimeB, freqHzB,
             freqHistoryB, freqIndexB, freqFilledB, false);
    calcFreq(samplesB, captureTimeB, freqHzB_hyst,
             freqHistoryB_hyst, freqIndexB_hyst, freqFilledB_hyst, true);
  }

  updateDisplayStats();

  // ===== DRAW =====

  display.clearDisplay();

  if (menuMode == 0) {
    // ---- WAVEFORM VIEW ----
    for (int i = 0; i < 128; i += 4)
      display.drawPixel(i, 32, WHITE);
    if (chA_active) drawWaveform(samplesA, adcOffsetA, ampScaleA, false);
    if (chB_active) drawWaveform(samplesB, adcOffsetB, ampScaleB, true);
    drawStatusBar();

  } else if (menuMode == 1) {
    // ---- STATS CH A ----
    if (chA_active) {
      drawStats('A', dispVppA, dispVmaxA, dispVminA, dispVdcA, dispFreqA, false);
    } else {
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(15, 28);
      display.print("CH A inactive");
    }

  } else if (menuMode == 2) {
    // ---- STATS CH B ----
    if (chB_active) {
      drawStats('B', dispVppB, dispVmaxB, dispVminB, dispVdcB, dispFreqB, false);
    } else {
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(15, 28);
      display.print("CH B inactive");
    }

  } else if (menuMode == 3) {
    // ---- HYSTERESIS WAVEFORM ----
    // Same waveform view but status bar shows [H] tag
    for (int i = 0; i < 128; i += 4)
      display.drawPixel(i, 32, WHITE);
    if (chA_active) drawWaveform(samplesA, adcOffsetA, ampScaleA, false);
    if (chB_active) drawWaveform(samplesB, adcOffsetB, ampScaleB, true);
    drawHystStatusBar();

  } else if (menuMode == 4) {
    // ---- HYSTERESIS VALUES ----
    if (chA_active && chB_active) {
      // Split screen
      display.setTextSize(1);
      display.setTextColor(WHITE);

      display.setCursor(0, 0);
      display.print("A[H] Vpp:");
      display.print(dispVppA, 1);
      display.print("V");

      display.setCursor(0, 9);
      display.print("  Freq:");
      if (dispFreqA_hyst >= 1000) {
        display.print(dispFreqA_hyst / 1000.0, 2);
        display.print("kHz");
      } else {
        display.print((int)dispFreqA_hyst);
        display.print("Hz");
      }

      display.setCursor(0, 18);
      display.print("  DC:");
      display.print(dispVdcA, 2);
      display.print("V");

      display.drawFastHLine(0, 30, 128, WHITE);

      display.setCursor(0, 33);
      display.print("B[H] Vpp:");
      display.print(dispVppB, 1);
      display.print("V");

      display.setCursor(0, 42);
      display.print("  Freq:");
      if (dispFreqB_hyst >= 1000) {
        display.print(dispFreqB_hyst / 1000.0, 2);
        display.print("kHz");
      } else {
        display.print((int)dispFreqB_hyst);
        display.print("Hz");
      }

      display.setCursor(0, 51);
      display.print("  DC:");
      display.print(dispVdcB, 2);
      display.print("V");

    } else if (chA_active) {
      drawStats('A', dispVppA, dispVmaxA, dispVminA, dispVdcA, dispFreqA_hyst, true);
    } else if (chB_active) {
      drawStats('B', dispVppB, dispVmaxB, dispVminB, dispVdcB, dispFreqB_hyst, true);
    } else {
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(20, 28);
      display.print("No ch active");
    }
  }

  display.display();
}