//Falcon Dashboard for ESP32 with ST7796S display
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7796S.h>

#define LCD_CS  13   // GPIO13
#define LCD_RS  14   // GPIO14 (data/command)
#define LCD_RST 27   // GPIO27
#define LCD_LED 26   // GPIO12 backlight
#define FUEL_ADC_PIN 25
#define SPEED_PULSE_PIN 32

// Tune this to match your speed sensor and tire setup.
// The GS-A-SS-2 is 16 pulses per revolution.
const float SPEED_PULSES_PER_REVOLUTION = 16.0f;
// Set this to your tire circumference in inches to get true MPH.
// If you leave it at 0, the display will show wheel RPM but MPH will stay 0.
const float WHEEL_CIRCUMFERENCE_INCHES = 0.0f;
const float SPEED_AXLE_RATIO = 3.73f;
const float SPEED_TIRE_REVOLUTIONS_PER_MILE = 800.0f;
const float SPEED_PULSES_PER_MILE = SPEED_PULSES_PER_REVOLUTION * SPEED_AXLE_RATIO * SPEED_TIRE_REVOLUTIONS_PER_MILE;
const float SPEED_TIRE_CIRCUMFERENCE_FEET = 6.5f;
const float SPEED_FEET_PER_MILE = 5280.0f;
const uint32_t SPEED_SAMPLE_WINDOW_MS = 500;
const uint32_t SPEED_DIRECT_SAMPLE_MS = 1000;
const uint32_t SPEED_MIN_PULSE_GAP_US = 120;
const int SPEED_INTERRUPT_MODE = CHANGE;
const bool SPEED_USE_INTERNAL_PULLUP = true;
const float SPEED_ADC_HIGH_THRESHOLD_V = 2.2f;
const float SPEED_ADC_LOW_THRESHOLD_V = 1.2f;
const uint32_t SPEED_IDLE_NOISE_PULSES = 0;
const uint8_t SPEED_GRAPH_POINTS = 48;
const float SPEED_FREQ_FILTER_ALPHA = 0.25f;
const float SPEED_ZERO_HOLD_HZ = 0.05f;
const float SPEED_IDLE_BIAS_HZ = 0.0f;
const float SPEED_CAL_DISPLAY_MPH_1 = 2.9f;
const float SPEED_CAL_TRUE_MPH_1 = 10.0f;
const float SPEED_CAL_DISPLAY_MPH_2 = 4.0f;
const float SPEED_CAL_TRUE_MPH_2 = 15.0f;
// Third calibration point derived from observed high-speed error.
// At true ~35 MPH the gauge showed ~42 MPH after 2-point calibration,
// which maps back to an estimated pre-calibration speed of ~9.94 MPH.
const float SPEED_CAL_DISPLAY_MPH_3 = 9.94f;
const float SPEED_CAL_TRUE_MPH_3 = 35.0f;
const uint32_t FUEL_SAMPLE_INTERVAL_MS = 60000;
const uint32_t FUEL_SUBSAMPLE_INTERVAL_MS = 5000;
const uint8_t FUEL_MODE_WINDOW_SAMPLES = (uint8_t)(FUEL_SAMPLE_INTERVAL_MS / FUEL_SUBSAMPLE_INTERVAL_MS);

#define BG_COLOR      0x0000  // black
#define LABEL_COLOR   0xFFFF  // white
#define VALUE_COLOR   0xFFE0  // yellow
#define BAR_COLOR     0x001F  // blue

int fuelPercent = 0;
float fuelVoltage = 0.0f;
int fuelSampleBuffer[FUEL_MODE_WINDOW_SAMPLES] = {0};
float fuelVoltageSampleBuffer[FUEL_MODE_WINDOW_SAMPLES] = {0.0f};
uint8_t fuelSampleCount = 0;
uint8_t fuelSampleWriteIndex = 0;
uint32_t lastFuelSubsampleMs = 0;
uint32_t lastFuelModeUpdateMs = 0;
volatile uint32_t speedPulseCount = 0;
volatile uint32_t speedDirectPulseCount = 0;
volatile uint32_t speedLastPulseUs = 0;
float speedMph = 0.0f;
float wheelRpm = 0.0f;
uint32_t lastSpeedSampleMs = 0;
uint32_t lastSpeedDirectSampleMs = 0;
uint32_t speedPulsesPerSample = 0;
float speedFrequencyHz = 0.0f;
float speedRawFrequencyHz = 0.0f;
float speedCorrectedFrequencyHz = 0.0f;
uint32_t speedRawPulsesLastSample = 0;
uint32_t speedUsedPulsesLastSample = 0;
float speedInputVoltage = 0.0f;
float speedDirectMph = 0.0f;
float speedDirectPulsesPerSecond = 0.0f;
volatile uint32_t speedAnalogPulseCount = 0;
bool speedAnalogPulseArmed = true;
float speedGraphMph[SPEED_GRAPH_POINTS] = {0.0f};
uint8_t speedGraphWriteIndex = 0;
uint8_t speedGraphCount = 0;

// Use the standard SPI hardware pins on ESP32:
// SCK  = 18, MOSI = 23, MISO = 19
Adafruit_ST7796S tft(LCD_CS, LCD_RS, LCD_RST);

float applySpeedCalibration(float mphMeasured) {
  const float x1 = SPEED_CAL_DISPLAY_MPH_1;
  const float y1 = SPEED_CAL_TRUE_MPH_1;
  const float x2 = SPEED_CAL_DISPLAY_MPH_2;
  const float y2 = SPEED_CAL_TRUE_MPH_2;
  const float x3 = SPEED_CAL_DISPLAY_MPH_3;
  const float y3 = SPEED_CAL_TRUE_MPH_3;

  const float span12 = x2 - x1;
  const float span23 = x3 - x2;
  if (fabsf(span12) < 0.001f || fabsf(span23) < 0.001f) {
    return mphMeasured;
  }

  float mphCalibrated = 0.0f;
  if (mphMeasured <= x2) {
    const float slope12 = (y2 - y1) / span12;
    mphCalibrated = y1 + slope12 * (mphMeasured - x1);
  } else {
    const float slope23 = (y3 - y2) / span23;
    mphCalibrated = y2 + slope23 * (mphMeasured - x2);
  }

  if (mphCalibrated < 0.0f) {
    mphCalibrated = 0.0f;
  }
  return mphCalibrated;
}

void updateSpeedAnalogPulseCounter() {
  int raw = analogRead(SPEED_PULSE_PIN);
  float volts = raw * (3.3f / 4095.0f);
  speedInputVoltage = volts;

  if (speedAnalogPulseArmed) {
    if (volts >= SPEED_ADC_HIGH_THRESHOLD_V) {
      speedAnalogPulseCount++;
      speedAnalogPulseArmed = false;
    }
  } else if (volts <= SPEED_ADC_LOW_THRESHOLD_V) {
    speedAnalogPulseArmed = true;
  }
}

void addSpeedGraphSample(float mph) {
  speedGraphMph[speedGraphWriteIndex] = mph;
  speedGraphWriteIndex = (speedGraphWriteIndex + 1) % SPEED_GRAPH_POINTS;
  if (speedGraphCount < SPEED_GRAPH_POINTS) {
    speedGraphCount++;
  }
}

void drawSpeedGraph(uint16_t x, uint16_t y, uint16_t w, uint16_t h, bool drawFrame = false) {
  if (drawFrame) {
    tft.drawRect(x, y, w, h, ST77XX_WHITE);
  }
  tft.fillRect(x + 1, y + 1, w - 2, h - 2, BG_COLOR);

  if (speedGraphCount < 2 || w < 4 || h < 4) {
    return;
  }

  float graphMax = 10.0f;
  for (uint8_t i = 0; i < speedGraphCount; i++) {
    uint8_t idx = (speedGraphWriteIndex + SPEED_GRAPH_POINTS - speedGraphCount + i) % SPEED_GRAPH_POINTS;
    if (speedGraphMph[idx] > graphMax) {
      graphMax = speedGraphMph[idx];
    }
  }

  float innerW = (float)(w - 3);
  float innerH = (float)(h - 3);
  int prevX = x + 1;
  uint8_t firstIdx = (speedGraphWriteIndex + SPEED_GRAPH_POINTS - speedGraphCount) % SPEED_GRAPH_POINTS;
  int prevY = y + (int)innerH - (int)((speedGraphMph[firstIdx] / graphMax) * innerH);

  for (uint8_t i = 1; i < speedGraphCount; i++) {
    uint8_t idx = (speedGraphWriteIndex + SPEED_GRAPH_POINTS - speedGraphCount + i) % SPEED_GRAPH_POINTS;
    int graphX = x + 1 + (int)((i * innerW) / (speedGraphCount - 1));
    int graphY = y + (int)innerH - (int)((speedGraphMph[idx] / graphMax) * innerH);
    tft.drawLine(prevX, prevY, graphX, graphY, ST77XX_CYAN);
    prevX = graphX;
    prevY = graphY;
  }
}

void addFuelSample(int percent, float volts) {
  fuelSampleBuffer[fuelSampleWriteIndex] = constrain(percent, 0, 100);
  fuelVoltageSampleBuffer[fuelSampleWriteIndex] = volts;
  fuelSampleWriteIndex = (fuelSampleWriteIndex + 1) % FUEL_MODE_WINDOW_SAMPLES;
  if (fuelSampleCount < FUEL_MODE_WINDOW_SAMPLES) {
    fuelSampleCount++;
  }
}

int getFuelModePercent(float* modeVoltageOut) {
  if (fuelSampleCount == 0) {
    if (modeVoltageOut != nullptr) {
      *modeVoltageOut = fuelVoltage;
    }
    return fuelPercent;
  }

  uint8_t counts[101] = {0};
  for (uint8_t i = 0; i < fuelSampleCount; i++) {
    int p = constrain(fuelSampleBuffer[i], 0, 100);
    counts[p]++;
  }

  int bestPercent = fuelSampleBuffer[0];
  uint8_t bestCount = 0;
  for (int p = 0; p <= 100; p++) {
    if (counts[p] > bestCount) {
      bestCount = counts[p];
      bestPercent = p;
    } else if (counts[p] == bestCount && bestCount > 0) {
      int currentDelta = abs(bestPercent - fuelPercent);
      int candidateDelta = abs(p - fuelPercent);
      if (candidateDelta < currentDelta) {
        bestPercent = p;
      }
    }
  }

  if (modeVoltageOut != nullptr) {
    float sumVolts = 0.0f;
    int matched = 0;
    for (uint8_t i = 0; i < fuelSampleCount; i++) {
      if (fuelSampleBuffer[i] == bestPercent) {
        sumVolts += fuelVoltageSampleBuffer[i];
        matched++;
      }
    }
    if (matched > 0) {
      *modeVoltageOut = sumVolts / matched;
    } else {
      *modeVoltageOut = fuelVoltage;
    }
  }

  return bestPercent;
}

void drawFordBootScreen() {
  tft.fillScreen(BG_COLOR);

  uint16_t w = tft.width();
  uint16_t h = tft.height();
  const uint16_t logoW = 260;
  const uint16_t logoH = 120;
  const uint16_t logoX = (w - logoW) / 2;
  const uint16_t logoY = (h - logoH) / 2 - 8;
  const uint8_t logoRadius = 58;

  tft.fillRoundRect(logoX, logoY, logoW, logoH, logoRadius, ST77XX_RED);
  tft.drawRoundRect(logoX, logoY, logoW, logoH, logoRadius, ST77XX_WHITE);
  tft.drawRoundRect(logoX + 2, logoY + 2, logoW - 4, logoH - 4, logoRadius - 2, ST77XX_WHITE);

  tft.setTextColor(ST77XX_WHITE, ST77XX_RED);
  tft.setTextSize(5);
  tft.setCursor(logoX + 66, logoY + 34);
  tft.print("Ford");

  tft.setTextColor(ST77XX_WHITE, BG_COLOR);
  tft.setTextSize(6);
  tft.setCursor((w / 2) - 220, logoY + logoH + 12);
  tft.print("Built Shitty");

  delay(5000);
}

void IRAM_ATTR countSpeedPulse() {
  speedDirectPulseCount++;

  uint32_t nowUs = micros();
  if ((nowUs - speedLastPulseUs) >= SPEED_MIN_PULSE_GAP_US) {
    speedPulseCount++;
    speedLastPulseUs = nowUs;
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(FUEL_ADC_PIN, INPUT);
  pinMode(SPEED_PULSE_PIN, SPEED_USE_INTERNAL_PULLUP ? INPUT_PULLUP : INPUT);
  pinMode(LCD_LED, OUTPUT);
  digitalWrite(LCD_LED, HIGH);  // turn on backlight
  attachInterrupt(digitalPinToInterrupt(SPEED_PULSE_PIN), countSpeedPulse, SPEED_INTERRUPT_MODE);

  // Configure SPI with the pins you specified.
  SPI.begin(18, 19, 23, 13);

  tft.init(320, 480);          // native ST7796S panel size
  tft.setRotation(1);           // landscape orientation
  tft.invertDisplay(true);      // required on this panel to show a black background
  tft.setTextWrap(false);
  tft.fillScreen(ST77XX_BLACK); // force black background

  drawFordBootScreen();
  drawDashboard();

  addFuelSample(fuelPercent, fuelVoltage);
  uint32_t now = millis();
  lastFuelSubsampleMs = now;
  lastFuelModeUpdateMs = now;

  lastSpeedSampleMs = millis();
  lastSpeedDirectSampleMs = lastSpeedSampleMs;
  speedLastPulseUs = micros();

  Serial.println("ST7796 display initialized.");
}

void drawPanel(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
               const char* title, const char* value,
               uint16_t borderColor, uint16_t fillColor) {
  const uint8_t cornerRadius = 8;
  const uint8_t borderThickness = 3;
  for (uint8_t i = 0; i < borderThickness; i++) {
    tft.drawRoundRect(x + i, y + i, w - 2 * i, h - 2 * i, cornerRadius - i, borderColor);
  }
  tft.fillRoundRect(x + borderThickness, y + borderThickness,
                    w - 2 * borderThickness, h - 2 * borderThickness,
                    cornerRadius - borderThickness, fillColor);

  tft.setTextColor(LABEL_COLOR, BG_COLOR);
  tft.setTextSize(1);
  tft.setCursor(x + 8, y + 8);
  tft.println(title);

  tft.setTextColor(VALUE_COLOR, BG_COLOR);
  tft.setTextSize(2);
  tft.setCursor(x + 8, y + 28);
  tft.println(value);
}

int readFuelPercent() {
  const float R_FIXED = 47.0f;
  const float V_REF = 3.3f;

  const float CAL_RES_OHM[5] = {78.0f, 50.0f, 26.0f, 16.0f, 10.0f};
  const float CAL_PERCENT[5] = {0.0f, 25.0f, 50.0f, 75.0f, 100.0f};

  int raw = analogRead(FUEL_ADC_PIN);
  float volts = raw * (V_REF / 4095.0f);
  fuelVoltage = volts;

  // Divider: 3.3V -> 47 ohm fixed resistor -> ADC node -> tank sender -> GND.
  // Vout = Vref * Rsender / (Rfixed + Rsender)
  // Rsender = Rfixed * Vout / (Vref - Vout)
  float senderOhms = 0.0f;
  if (volts >= (V_REF - 0.001f)) {
    senderOhms = 1000.0f;  // fail-safe clamp near divider singularity
  } else {
    senderOhms = (R_FIXED * volts) / (V_REF - volts);
  }

  // Non-linear sender calibration (ohms -> %):
  // 78=E, 50=1/4, 26=1/2, 16=3/4, 10=F.
  float percent = 0.0f;
  if (senderOhms >= CAL_RES_OHM[0]) {
    percent = CAL_PERCENT[0];
  } else if (senderOhms <= CAL_RES_OHM[4]) {
    percent = CAL_PERCENT[4];
  } else {
    for (int i = 0; i < 4; i++) {
      if (senderOhms <= CAL_RES_OHM[i] && senderOhms >= CAL_RES_OHM[i + 1]) {
        float spanOhms = CAL_RES_OHM[i] - CAL_RES_OHM[i + 1];
        float t = (CAL_RES_OHM[i] - senderOhms) / spanOhms;
        percent = CAL_PERCENT[i] + t * (CAL_PERCENT[i + 1] - CAL_PERCENT[i]);
        break;
      }
    }
  }
  percent = constrain(percent, 0.0f, 100.0f);

  int fuelPercentValue = (int)round(percent);

  Serial.print("Fuel ADC raw=");
  Serial.print(raw);
  Serial.print("  volts=");
  Serial.print(volts, 3);
  Serial.print("V  senderOhms=");
  Serial.print(senderOhms, 1);
  Serial.print("  fuel=");
  Serial.print(fuelPercentValue);
  Serial.println("% ");

  return fuelPercentValue;
}

void drawFuelPanel(int percent) {
  uint16_t w = tft.width();
  const uint16_t gap = 6;
  const uint16_t panelW = (w - 3 * gap) / 2;
  const uint16_t panelH = 92;
  const uint16_t bottomY = tft.height() / 2 + 2;
  (void)panelH;
  const float tankGallons = 10.0f * (percent / 100.0f);

  char fuelText[24];
  char gallonsText[24];
  char voltageText[24];
  snprintf(fuelText, sizeof(fuelText), "%d %%", percent);
  snprintf(gallonsText, sizeof(gallonsText), "%.1f gal", tankGallons);
  snprintf(voltageText, sizeof(voltageText), "%.3f V", fuelVoltage);

  // Keep panel border static; only refresh value regions.
  tft.setTextColor(VALUE_COLOR, BG_COLOR);
  tft.setTextSize(2);
  tft.fillRect(gap + 8, bottomY + 28, panelW - 16, 16, BG_COLOR);
  tft.setCursor(gap + 8, bottomY + 28);
  tft.println(fuelText);

  tft.setTextColor(ST77XX_WHITE, BG_COLOR);
  tft.setTextSize(1);
  tft.fillRect(gap + 8, bottomY + 58, panelW - 16, 18, BG_COLOR);
  tft.setCursor(gap + 8, bottomY + 58);
  tft.println(gallonsText);

  tft.setCursor(gap + 8, bottomY + 68);
  tft.println(voltageText);

  // Draw a horizontal fuel gauge bar under the text.
  const uint16_t gaugeX = gap + 8;
  const uint16_t gaugeY = bottomY + 74;
  const uint16_t gaugeW = panelW - 16;
  const uint16_t gaugeH = 16;
  const uint16_t gaugeFillW = (uint16_t)((gaugeW - 2) * (percent / 100.0f));
  uint16_t gaugeColor = ST77XX_GREEN;
  if (percent < 25) {
    gaugeColor = ST77XX_RED;
  } else if (percent <= 50) {
    gaugeColor = ST77XX_YELLOW;
  }

  tft.fillRect(gaugeX + 1, gaugeY + 1, gaugeW - 2, gaugeH - 2, BG_COLOR);
  if (gaugeFillW > 0) {
    tft.fillRect(gaugeX + 1, gaugeY + 1, gaugeFillW, gaugeH - 2, gaugeColor);
  }

  // Quarter segment markers (25%, 50%, 75%).
  const uint16_t marker1X = gaugeX + 1 + (uint16_t)(((gaugeW - 2) * 1) / 4);
  const uint16_t marker2X = gaugeX + 1 + (uint16_t)(((gaugeW - 2) * 2) / 4);
  const uint16_t marker3X = gaugeX + 1 + (uint16_t)(((gaugeW - 2) * 3) / 4);
  tft.drawFastVLine(marker1X, gaugeY + 1, gaugeH - 2, ST77XX_WHITE);
  tft.drawFastVLine(marker2X, gaugeY + 1, gaugeH - 2, ST77XX_WHITE);
  tft.drawFastVLine(marker3X, gaugeY + 1, gaugeH - 2, ST77XX_WHITE);
}

void drawSpeedPanel(float mph, float rpm, bool fullRedraw = false) {
  (void)rpm;
  (void)fullRedraw;

  uint16_t w = tft.width();
  const uint16_t gap = 6;
  const uint16_t panelW = (w - 3 * gap) / 2;
  const uint16_t panelH = 92;
  const uint16_t topY = 6;
  const uint16_t graphW = 34;
  const uint16_t graphX = gap + panelW - 8 - graphW;
  const uint16_t graphY = topY + 20;
  const uint16_t graphH = panelH - 24;
  const uint16_t numberX = gap + 8;
  const uint16_t numberW = graphX - numberX - 6;

  char speedText[12];
  int speedRounded = (int)roundf(mph);
  snprintf(speedText, sizeof(speedText), "%d", speedRounded);

  addSpeedGraphSample(mph);

  tft.setTextColor(VALUE_COLOR, BG_COLOR);
  tft.setTextSize(5);
  int16_t textX = numberX + ((int16_t)numberW - ((int16_t)strlen(speedText) * 6 * 5)) / 2;
  if (textX < (int16_t)numberX) {
    textX = numberX;
  }
  tft.fillRect(numberX, topY + 18, numberW, 34, BG_COLOR);
  tft.setCursor(textX, topY + 20);
  tft.print(speedText);

  drawSpeedGraph(graphX, graphY, graphW, graphH, false);
}

void drawDashboard() {
  uint16_t w = tft.width();
  uint16_t h = tft.height();
  fuelPercent = readFuelPercent();

  tft.fillScreen(BG_COLOR);  // force black background every time

  const uint16_t gap = 6;
  const uint16_t panelH = 92;
  const uint16_t topY = 6;
  const uint16_t bottomY = h / 2 + 2;
  const uint16_t panelW = (w - 3 * gap) / 2;
  const uint16_t speedGraphW = 34;
  const uint16_t speedGraphX = gap + panelW - 8 - speedGraphW;
  const uint16_t speedGraphY = topY + 20;
  const uint16_t speedGraphH = panelH - 24;

  drawPanel(gap, topY, panelW, panelH, "SPEED", "", 0x07FF, BG_COLOR);
  drawPanel(w - gap - panelW, topY, panelW, panelH, "RPM", "0", 0xF800, BG_COLOR);
  drawPanel(gap, bottomY, panelW, panelH, "FUEL", "", 0x07E0, BG_COLOR);
  drawPanel(w - gap - panelW, bottomY, panelW, panelH, "TEMP", "68 *F", 0xF81F, BG_COLOR);

  // Divider lines for the physical 480x320 landscape screen
  tft.drawFastVLine(w / 2, 0, h, ST77XX_WHITE);
  tft.drawFastHLine(0, h / 2, w, ST77XX_WHITE);

  // Bottom status bar
  const uint16_t barH = 22;
  tft.fillRect(0, h - barH, w, barH, BAR_COLOR);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  tft.setCursor(8, h - 15);
  tft.println("CAR DASHBOARD  -  Falcon ESP32");

  // Draw static inner frames once, then only update dynamic values.
  drawSpeedGraph(speedGraphX, speedGraphY, speedGraphW, speedGraphH, true);
  const uint16_t gaugeX = gap + 8;
  const uint16_t gaugeY = bottomY + 74;
  const uint16_t gaugeW = panelW - 16;
  const uint16_t gaugeH = 16;
  tft.drawRect(gaugeX, gaugeY, gaugeW, gaugeH, ST77XX_WHITE);

  drawSpeedPanel(speedMph, wheelRpm, true);
  drawFuelPanel(fuelPercent);
}

void loop() {
  uint32_t now = millis();

  // Simple pulse counting path using ADC threshold crossings.
  updateSpeedAnalogPulseCounter();

  if (now - lastSpeedDirectSampleMs >= SPEED_DIRECT_SAMPLE_MS) {
    noInterrupts();
    uint32_t directPulses = speedDirectPulseCount;
    speedDirectPulseCount = 0;
    uint32_t analogPulses = speedAnalogPulseCount;
    speedAnalogPulseCount = 0;
    interrupts();

    if (directPulses == 0 && analogPulses > 0) {
      directPulses = analogPulses;
    }

    uint32_t directElapsedMs = now - lastSpeedDirectSampleMs;
    lastSpeedDirectSampleMs = now;

    if (directElapsedMs > 0) {
      float elapsedSeconds = directElapsedMs / 1000.0f;
      speedDirectPulsesPerSecond = directPulses / elapsedSeconds;

      float rps = speedDirectPulsesPerSecond / SPEED_PULSES_PER_REVOLUTION;
      float fps = rps * SPEED_TIRE_CIRCUMFERENCE_FEET;
      speedDirectMph = (fps * 3600.0f) / SPEED_FEET_PER_MILE;
    }

    Serial.print("Direct pulses/s=");
    Serial.print(speedDirectPulsesPerSecond, 1);
    Serial.print("  direct mph=");
    Serial.println(speedDirectMph, 1);
  }

  if (now - lastFuelSubsampleMs >= FUEL_SUBSAMPLE_INTERVAL_MS) {
    lastFuelSubsampleMs = now;
    int subsamplePercent = readFuelPercent();
    addFuelSample(subsamplePercent, fuelVoltage);
  }

  if (now - lastFuelModeUpdateMs >= FUEL_SAMPLE_INTERVAL_MS) {
    lastFuelModeUpdateMs = now;
    float modeVoltage = fuelVoltage;
    int newFuelPercent = getFuelModePercent(&modeVoltage);
    fuelVoltage = modeVoltage;

    if (newFuelPercent != fuelPercent) {
      fuelPercent = newFuelPercent;
    }

    drawFuelPanel(fuelPercent);
  }

  if (now - lastSpeedSampleMs >= SPEED_SAMPLE_WINDOW_MS) {
    noInterrupts();
    uint32_t pulseCount = speedPulseCount;
    speedPulseCount = 0;
    interrupts();

    const uint32_t rawPulseCount = pulseCount;
    speedRawPulsesLastSample = rawPulseCount;
    if (pulseCount <= SPEED_IDLE_NOISE_PULSES) {
      pulseCount = 0;
    }
    speedUsedPulsesLastSample = pulseCount;

    uint32_t elapsedMs = now - lastSpeedSampleMs;
    float elapsedHours = elapsedMs / 3600000.0f;
    lastSpeedSampleMs = now;

    if (elapsedHours > 0.0f) {
      speedRawFrequencyHz = pulseCount * (1000.0f / elapsedMs);
      speedCorrectedFrequencyHz = speedRawFrequencyHz - SPEED_IDLE_BIAS_HZ;
      if (speedCorrectedFrequencyHz < 0.0f) {
        speedCorrectedFrequencyHz = 0.0f;
      }

      speedFrequencyHz = (SPEED_FREQ_FILTER_ALPHA * speedCorrectedFrequencyHz) +
                         ((1.0f - SPEED_FREQ_FILTER_ALPHA) * speedFrequencyHz);

      if (speedFrequencyHz < SPEED_ZERO_HOLD_HZ) {
        speedFrequencyHz = 0.0f;
      }

      // Use direct counted pulses/sec when available; this path is proving reliable.
      float effectiveFrequencyHz = speedFrequencyHz;
      if (speedDirectPulsesPerSecond > effectiveFrequencyHz) {
        effectiveFrequencyHz = speedDirectPulsesPerSecond;
      }
      speedFrequencyHz = effectiveFrequencyHz;

      speedPulsesPerSample = (uint32_t)((effectiveFrequencyHz * elapsedMs / 1000.0f) + 0.5f);

      float filteredRevolutions = (effectiveFrequencyHz * elapsedMs / 1000.0f) / SPEED_PULSES_PER_REVOLUTION;
      wheelRpm = filteredRevolutions / elapsedHours / 60.0f;

      if (SPEED_PULSES_PER_MILE > 0.0f) {
        speedMph = (effectiveFrequencyHz * 3600.0f) / SPEED_PULSES_PER_MILE;
      } else if (WHEEL_CIRCUMFERENCE_INCHES > 0.0f) {
        speedMph = (filteredRevolutions / elapsedHours) * (WHEEL_CIRCUMFERENCE_INCHES / 63360.0f);
      } else {
        speedMph = 0.0f;
      }

      // If the filtered pipeline is still at zero, show direct counter MPH for diagnosis.
      if (speedMph <= 0.0f && speedDirectMph > 0.0f) {
        speedMph = speedDirectMph;
      }

      speedMph = applySpeedCalibration(speedMph);
    }

    if (pulseCount == 0 && speedDirectPulsesPerSecond <= 0.1f) {
      speedFrequencyHz *= 0.7f;
      if (speedFrequencyHz < SPEED_ZERO_HOLD_HZ) {
        wheelRpm = 0.0f;
        speedMph = 0.0f;
        speedFrequencyHz = 0.0f;
        speedCorrectedFrequencyHz = 0.0f;
        speedPulsesPerSample = 0;
      }
    }

    Serial.print("Speed pulses raw=");
    Serial.print((unsigned long)rawPulseCount);
    Serial.print(" used=");
    Serial.print((unsigned long)pulseCount);
    Serial.print("  raw=");
    Serial.print(speedRawFrequencyHz, 1);
    Serial.print("  corr=");
    Serial.print(speedCorrectedFrequencyHz, 1);
    Serial.print("  freq=");
    Serial.print(speedFrequencyHz, 1);
    Serial.print(" Hz");
    Serial.print("  rpm=");
    Serial.print(wheelRpm, 1);
    Serial.print("  mph=");
    Serial.println(speedMph, 0);

    drawSpeedPanel(speedMph, wheelRpm, true);
  }
}
