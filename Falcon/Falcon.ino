//Falcon Dashboard for ESP32 with ST7796S display
#include <SPI.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7796S.h>
#include <Fonts/FreeSansBoldOblique24pt7b.h>

#define LCD_CS  13   // GPIO13
#define LCD_RS  14   // GPIO14 (data/command)
#define LCD_RST 27   // GPIO27
#define LCD_LED 26   // GPIO12 backlight
#define FUEL_ADC_PIN 25
#define SPEED_PULSE_PIN 32

// Second ST7796S display. Shares SCK/MOSI with the first display;
// only CS/DC/RST/backlight need to be unique per panel.
#define LCD2_CS  33  // GPIO33
#define LCD2_RS  4   // GPIO4  (data/command)
#define LCD2_RST 21  // GPIO21
#define LCD2_LED 22  // GPIO22 backlight

// If boot graphics ever cause instability, set false to skip the animated screen.
const bool SHOW_BOOT_SCREEN = true;

// Tune this to match your speed sensor and tire setup.
// The GS-A-SS-2 is 16 pulses per revolution.
const float SPEED_PULSES_PER_REVOLUTION = 16.0f;
// Set this to your tire circumference in inches to get true MPH.
// If you leave it at 0, the display will show wheel RPM but MPH will stay 0.
const float WHEEL_CIRCUMFERENCE_INCHES = 0.0f;
const float SPEED_TIRE_REVOLUTIONS_PER_MILE = 800.0f;
const float SPEED_PULSES_PER_MILE = SPEED_PULSES_PER_REVOLUTION * SPEED_TIRE_REVOLUTIONS_PER_MILE;
const float SPEED_TIRE_CIRCUMFERENCE_FEET = 6.5f;
const float SPEED_FEET_PER_MILE = 5280.0f;
const uint32_t SPEED_SAMPLE_WINDOW_MS = 500;
const uint32_t SPEED_DIRECT_SAMPLE_MS = 1000;
const uint32_t SPEED_MIN_PULSE_GAP_US = 120;
// Require more than a single stray pulse in the direct-sample window before trusting it as real speed.
const uint32_t SPEED_DIRECT_MIN_PULSES = 2;
const int SPEED_INTERRUPT_MODE = RISING;
const bool SPEED_USE_INTERNAL_PULLUP = true;
const uint32_t SPEED_IDLE_NOISE_PULSES = 0;
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
const float SPEED_DIAL_MAX_MPH = 100.0f;
const uint16_t DASH_TOP_OFFSET_PX = 20;
const uint32_t FUEL_SAMPLE_INTERVAL_MS = 120000;
const uint32_t FUEL_SUBSAMPLE_INTERVAL_MS = 1000;
const uint8_t FUEL_MODE_WINDOW_SAMPLES = (uint8_t)(FUEL_SAMPLE_INTERVAL_MS / FUEL_SUBSAMPLE_INTERVAL_MS);
const char* ODOMETER_PREF_NAMESPACE = "falcon";
const char* ODOMETER_PREF_KEY = "odo_mi";
const float ODOMETER_BASELINE_MILES = 88384.0f;
const float ODOMETER_SAVE_DELTA_MILES = 0.01f;
const uint32_t ODOMETER_SAVE_INTERVAL_MS = 15000;
const bool ODOMETER_PAUSED = true;

// This ST7796S panel displays non-inverted RGB565 values with inverted visible
// colors. Keep controller inversion disabled and compensate colors at draw time.
#define DISPLAY_BLACK      0xFFFF
#define DISPLAY_WHITE      0x0000
#define DISPLAY_RED        0x07FF
#define DISPLAY_GREEN      0xF81F
#define DISPLAY_BLUE       0xFFE0
#define DISPLAY_YELLOW     0x001F
#define DISPLAY_GRAY       0x8410
#define DISPLAY_FORD_BLUE  0xC569
#define WARNING_COLOR      0xF800
#define NEEDLE_COLOR       DISPLAY_BLUE

#define BG_COLOR      DISPLAY_BLACK
#define LABEL_COLOR   DISPLAY_WHITE
#define VALUE_COLOR   DISPLAY_YELLOW
#define BAR_COLOR     DISPLAY_BLUE

#undef ST77XX_BLACK
#undef ST77XX_WHITE
#undef ST77XX_RED
#undef ST77XX_GREEN
#undef ST77XX_BLUE
#undef ST77XX_YELLOW
#define ST77XX_BLACK  DISPLAY_BLACK
#define ST77XX_WHITE  DISPLAY_WHITE
#define ST77XX_RED    DISPLAY_RED
#define ST77XX_GREEN  DISPLAY_GREEN
#define ST77XX_BLUE   DISPLAY_BLUE
#define ST77XX_YELLOW DISPLAY_YELLOW

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
float speedDirectMph = 0.0f;
float speedDirectPulsesPerSecond = 0.0f;
float odometerMiles = 0.0f;
int16_t speedNeedlePrevX = -1;
int16_t speedNeedlePrevY = -1;
int16_t coolantNeedlePrevX = -1;
int16_t coolantNeedlePrevY = -1;
int16_t oilNeedlePrevX = -1;
int16_t oilNeedlePrevY = -1;
int16_t batteryNeedlePrevX = -1;
int16_t batteryNeedlePrevY = -1;
float coolantTempF = 68.0f;
float oilPsi = 0.0f;
float batteryVoltage = 12.6f;

const float COOLANT_DIAL_MIN_F = 100.0f;
const float COOLANT_DIAL_MAX_F = 260.0f;
const float OIL_PSI_MIN = 0.0f;
const float OIL_PSI_MAX = 100.0f;
const float BATTERY_DIAL_MIN_V = 10.0f;
const float BATTERY_DIAL_MAX_V = 16.0f;
const float COOLANT_LOW_WARNING_F = 160.0f;
const float OIL_LOW_WARNING_PSI = 10.0f;
const int16_t SCREEN1_GAUGE_RADIUS = 100;
const int16_t SCREEN1_GAUGE_TOP_CENTER_Y = 103;
const int16_t SCREEN1_GAUGE_BOTTOM_CENTER_Y = 255;
const float SCREEN1_GAUGE_START_DEG = 180.0f;
const float SCREEN1_GAUGE_END_DEG = 270.0f;

// Use the standard SPI hardware pins on ESP32:
// SCK  = 18, MOSI = 23, MISO = 19
Adafruit_ST7796S tft(LCD_CS, LCD_RS, LCD_RST);

// Second display, same SPI bus, its own CS/DC/RST.
Adafruit_ST7796S tft2(LCD2_CS, LCD2_RS, LCD2_RST);
Preferences preferences;

int16_t fuelNeedlePrevX = -1;
int16_t fuelNeedlePrevY = -1;
float odometerSavedMiles = 0.0f;
uint32_t lastOdometerSaveMs = 0;
SemaphoreHandle_t displayMutex = NULL;
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t displayTaskHandle = NULL;

int lastRenderedFuelPercent = INT_MIN;
float lastRenderedFuelVoltage = -999.0f;
float lastRenderedBatteryVoltage = -999.0f;
float lastRenderedCoolantTempF = -999.0f;
float lastRenderedOilPsi = -999.0f;
float lastRenderedSpeedMph = -999.0f;
float lastRenderedOdometerMiles = -999.0f;

void drawFuelDial(int percent, bool fullRedraw = false);
void drawCoolantDial(float tempF, bool fullRedraw = false);
void drawOilPsiDial(float psi, bool fullRedraw = false);
void drawBatteryDial(float volts, bool fullRedraw = false);
void drawFuelVoltageReadout(float volts);
void drawLowFuelIndicator(bool visible);
void drawGaugeWarningIndicator(int16_t warningY, const char* label, bool visible);
void drawGaugeLeftLabel(int16_t leftX, int16_t leftY, const char* label);
void drawOdometerPanel(float miles, bool fullRedraw = false);
void drawScreen_1(void);
void drawScreen_2(void);
void drawDashboard(void);
void renderDashboardFrame(void);
void loadOdometer(void);
void saveOdometerIfNeeded(bool forceSave = false);
void sensorTask(void* parameter);
void displayTask(void* parameter);

void drawTaperedNeedle(Adafruit_GFX& display, int16_t pivotX, int16_t pivotY,
                       int16_t tipX, int16_t tipY, uint16_t color) {
  const float dx = (float)(tipX - pivotX);
  const float dy = (float)(tipY - pivotY);
  const float length = sqrtf((dx * dx) + (dy * dy));
  if (length <= 0.1f) {
    return;
  }

  const float baseHalfWidth = 5.0f;
  const float tipHalfWidth = 2.0f;
  const float perpendicularX = -dy / length;
  const float perpendicularY = dx / length;

  const int16_t baseLeftX = pivotX + (int16_t)roundf(perpendicularX * baseHalfWidth);
  const int16_t baseLeftY = pivotY + (int16_t)roundf(perpendicularY * baseHalfWidth);
  const int16_t baseRightX = pivotX - (int16_t)roundf(perpendicularX * baseHalfWidth);
  const int16_t baseRightY = pivotY - (int16_t)roundf(perpendicularY * baseHalfWidth);
  const int16_t tipLeftX = tipX + (int16_t)roundf(perpendicularX * tipHalfWidth);
  const int16_t tipLeftY = tipY + (int16_t)roundf(perpendicularY * tipHalfWidth);
  const int16_t tipRightX = tipX - (int16_t)roundf(perpendicularX * tipHalfWidth);
  const int16_t tipRightY = tipY - (int16_t)roundf(perpendicularY * tipHalfWidth);

  display.fillTriangle(baseLeftX, baseLeftY, baseRightX, baseRightY,
                       tipRightX, tipRightY, color);
  display.fillTriangle(baseLeftX, baseLeftY, tipRightX, tipRightY,
                       tipLeftX, tipLeftY, color);
}

void logCheckpoint(const char* msg) {
  Serial.print("[BOOT] ");
  Serial.println(msg);
}

void loadOdometer(void) {
  preferences.begin(ODOMETER_PREF_NAMESPACE, false);
  odometerMiles = preferences.getFloat(ODOMETER_PREF_KEY, ODOMETER_BASELINE_MILES);
  if (!isfinite(odometerMiles) || odometerMiles < ODOMETER_BASELINE_MILES) {
    odometerMiles = ODOMETER_BASELINE_MILES;
  }
  odometerSavedMiles = odometerMiles;
  lastOdometerSaveMs = millis();
}

void saveOdometerIfNeeded(bool forceSave) {
  float deltaMiles = fabsf(odometerMiles - odometerSavedMiles);
  uint32_t now = millis();

  if (!forceSave && deltaMiles < ODOMETER_SAVE_DELTA_MILES &&
      (now - lastOdometerSaveMs) < ODOMETER_SAVE_INTERVAL_MS) {
    return;
  }

  preferences.putFloat(ODOMETER_PREF_KEY, odometerMiles);
  odometerSavedMiles = odometerMiles;
  lastOdometerSaveMs = now;
}

void prepareDisplayPins() {
  // Keep both displays deselected on the shared SPI bus during init.
  pinMode(LCD_CS, OUTPUT);
  pinMode(LCD2_CS, OUTPUT);
  digitalWrite(LCD_CS, HIGH);
  digitalWrite(LCD2_CS, HIGH);

  pinMode(LCD_RS, OUTPUT);
  pinMode(LCD2_RS, OUTPUT);
  pinMode(LCD_RST, OUTPUT);
  pinMode(LCD2_RST, OUTPUT);

  // Apply a deterministic hardware reset to both controllers.
  digitalWrite(LCD_RST, HIGH);
  digitalWrite(LCD2_RST, HIGH);
  delay(5);
  digitalWrite(LCD_RST, LOW);
  digitalWrite(LCD2_RST, LOW);
  delay(15);
  digitalWrite(LCD_RST, HIGH);
  digitalWrite(LCD2_RST, HIGH);
  delay(120);
}

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

  // Keep this boot art simple and primitive-only to avoid library-specific issues.
  const int16_t logoCenterX = 239;
  const int16_t logoCenterY = 153;
  // Flatter/wider capsule geometry to better approximate an ellipse.
  const int16_t logoOuterX = 10;
  const int16_t logoOuterY = 44;
  const int16_t logoOuterW = 460;
  const int16_t logoOuterH = 218;
  const int16_t logoOuterR = 108;
  const int16_t logoInnerInset = 8;
  const int16_t logoInnerR = 100;
  // Use the natural Ford blue background for the non-inverted display.
  tft.fillRoundRect(logoOuterX, logoOuterY, logoOuterW, logoOuterH, logoOuterR, DISPLAY_FORD_BLUE);
  tft.drawRoundRect(logoOuterX, logoOuterY, logoOuterW, logoOuterH, logoOuterR, DISPLAY_WHITE);
  tft.drawRoundRect(logoOuterX + logoInnerInset, logoOuterY + logoInnerInset,
                    logoOuterW - (logoInnerInset * 2), logoOuterH - (logoInnerInset * 2),
                    logoInnerR, DISPLAY_WHITE);

  tft.setTextColor(DISPLAY_WHITE, DISPLAY_FORD_BLUE);
  tft.setTextSize(1);
  tft.setFont(&FreeSansBoldOblique24pt7b);
  int16_t textX1 = 0;
  int16_t textY1 = 0;
  uint16_t textW = 0;
  uint16_t textH = 0;
  tft.getTextBounds("Ford", 0, 0, &textX1, &textY1, &textW, &textH);
  int16_t textCursorX = logoCenterX - (textX1 + ((int16_t)textW / 2));
  int16_t textCursorY = logoCenterY - (textY1 + ((int16_t)textH / 2));
  tft.setCursor(textCursorX, textCursorY);
  tft.print("Ford");
  tft.setFont();

  tft.setTextColor(DISPLAY_WHITE, BG_COLOR);
  tft.setTextSize(3);
  tft.setCursor(18, 244);
  tft.print("Built Shitty");

  // Second screen boot loader with progress bar during startup hold.
  tft2.fillScreen(BG_COLOR);
  uint16_t w2 = tft2.width();
  uint16_t h2 = tft2.height();
  const uint16_t panelX = 22;
  const uint16_t panelY = 78;
  const uint16_t panelW = w2 - 44;
  const uint16_t panelH = h2 - 156;
  const uint16_t barX = panelX + 18;
  const uint16_t barY = panelY + 76;
  const uint16_t barW = panelW - 36;
  const uint16_t barH = 24;

  tft2.drawRoundRect(panelX, panelY, panelW, panelH, 10, DISPLAY_WHITE);
  tft2.setTextColor(DISPLAY_WHITE, BG_COLOR);
  tft2.setTextSize(2);
  tft2.setCursor(panelX + 28, panelY + 20);
  tft2.print("SECONDARY DISPLAY");
  tft2.setTextSize(2);
  tft2.setCursor(panelX + 54, panelY + 44);
  tft2.print("INITIALIZING");

  tft2.drawRoundRect(barX, barY, barW, barH, 6, DISPLAY_WHITE);

  const uint16_t barInnerW = barW - 4;
  for (int percent = 0; percent <= 100; percent++) {
    uint16_t fillW = (uint16_t)((barInnerW * percent) / 100);
    tft2.fillRect(barX + 2, barY + 2, barInnerW, barH - 4, BG_COLOR);
    if (fillW > 0) {
      tft2.fillRect(barX + 2, barY + 2, fillW, barH - 4, DISPLAY_GREEN);
    }

    char percentText[10];
    snprintf(percentText, sizeof(percentText), "%3d%%", percent);
    tft2.fillRect(panelX + 96, barY + 32, 80, 16, BG_COLOR);
    tft2.setTextColor(VALUE_COLOR, BG_COLOR);
    tft2.setTextSize(2);
    tft2.setCursor(panelX + 100, barY + 34);
    tft2.print(percentText);

    delay(50);
  }
}

void IRAM_ATTR countSpeedPulse() {
  // Debounce both counters identically; without this a single noisy edge
  // (e.g. CHANGE-mode bounce) reports nonzero speed even at true 0 MPH.
  uint32_t nowUs = micros();
  if ((nowUs - speedLastPulseUs) >= SPEED_MIN_PULSE_GAP_US) {
    speedPulseCount++;
    speedDirectPulseCount++;
    speedLastPulseUs = nowUs;
  }
}

void setup() {
  Serial.begin(115200);
  delay(250);
  logCheckpoint("Serial online");

  loadOdometer();
  logCheckpoint("Odometer restored");

  pinMode(FUEL_ADC_PIN, INPUT);
  pinMode(SPEED_PULSE_PIN, SPEED_USE_INTERNAL_PULLUP ? INPUT_PULLUP : INPUT);
  pinMode(LCD_LED, OUTPUT);
  pinMode(LCD2_LED, OUTPUT);
  digitalWrite(LCD_LED, LOW);
  digitalWrite(LCD2_LED, LOW);

  logCheckpoint("Preparing display pins");
  prepareDisplayPins();

  // Configure SPI with the pins you specified.
  logCheckpoint("Starting SPI bus");
  SPI.begin(18, 19, 23, 13);

  logCheckpoint("Init display 1");
  tft.init(320, 480);          // native ST7796S panel size
  tft.setRotation(1);           // landscape orientation
  tft.invertDisplay(false);     // use the panel's natural color mapping and fill black explicitly
  tft.setTextWrap(false);
  tft.fillScreen(BG_COLOR);     // black background, no controller inversion required

  logCheckpoint("Init display 2");
  tft2.init(320, 480);          // native ST7796S panel size
  tft2.setRotation(3);          // landscape orientation, flipped 180 from tft
  tft2.invertDisplay(false);    // use the panel's natural color mapping and fill black explicitly
  tft2.setTextWrap(false);
  tft2.fillScreen(BG_COLOR);    // black background, no controller inversion required

  digitalWrite(LCD_LED, HIGH);   // turn on backlight after panels are initialized
  digitalWrite(LCD2_LED, HIGH);  // turn on second display backlight

  displayMutex = xSemaphoreCreateMutex();

  logCheckpoint("Drawing initial UI");
  if (SHOW_BOOT_SCREEN) {
    drawFordBootScreen();
  }

  // Clear both panels after boot art so each gauge layout starts clean.
  tft.fillScreen(BG_COLOR);
  drawScreen_1();
  tft2.fillScreen(BG_COLOR);
  drawScreen_2();

  drawDashboard();
  drawFuelDial(fuelPercent, true);
  drawCoolantDial(coolantTempF, true);
  drawOilPsiDial(oilPsi, true);
  drawBatteryDial(batteryVoltage, true);
  drawFuelVoltageReadout(fuelVoltage);

  addFuelSample(fuelPercent, fuelVoltage);
  uint32_t now = millis();
  lastFuelSubsampleMs = now;
  lastFuelModeUpdateMs = now;

  lastSpeedSampleMs = millis();
  lastSpeedDirectSampleMs = lastSpeedSampleMs;
  speedLastPulseUs = micros();

  xTaskCreatePinnedToCore(sensorTask, "sensorTask", 4096, NULL, 2, &sensorTaskHandle, 1);
  xTaskCreatePinnedToCore(displayTask, "displayTask", 4096, NULL, 1, &displayTaskHandle, 0);

  // Attach interrupt only after display init and first full draw are complete.
  attachInterrupt(digitalPinToInterrupt(SPEED_PULSE_PIN), countSpeedPulse, SPEED_INTERRUPT_MODE);
  logCheckpoint("Speed interrupt attached");

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

  // Keep empty calibration point as-is, but set full-scale to 0.7V.
  // Convert the desired full voltage to equivalent sender resistance.
  const float FULL_SCALE_VOLTS = 0.7f;
  const float FULL_SCALE_RES_OHM = (R_FIXED * FULL_SCALE_VOLTS) / (V_REF - FULL_SCALE_VOLTS);

  const float CAL_RES_OHM[5] = {78.0f, 50.0f, 26.0f, 16.0f, FULL_SCALE_RES_OHM};
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

void drawTempPanel(float tempF) {
  uint16_t w = tft.width();
  uint16_t h = tft.height();
  const uint16_t outerGap = 6;
  const uint16_t topGap = outerGap + DASH_TOP_OFFSET_PX;
  const uint16_t splitGap = 8;
  const uint16_t rightGap = 6;
  const uint16_t barH = 0;
  const uint16_t contentH = h - topGap - outerGap - barH;
  const uint16_t leftW = (uint16_t)(((w - 2 * outerGap - splitGap) * 55) / 100);
  const uint16_t rightX = outerGap + leftW + splitGap;
  const uint16_t rightW = w - outerGap - rightX;
  const uint16_t rightPanelH = (contentH - rightGap) / 2;
  const uint16_t tempY = topGap;

  char tempText[24];
  snprintf(tempText, sizeof(tempText), "%.0f F", tempF);

  tft.setTextColor(VALUE_COLOR, BG_COLOR);
  tft.setTextSize(3);
  tft.fillRect(rightX + 8, tempY + 22, rightW - 16, 28, BG_COLOR);
  tft.setCursor(rightX + 8, tempY + 24);
  tft.print(tempText);

  tft.setTextColor(DISPLAY_WHITE, BG_COLOR);
  tft.setTextSize(1);
  tft.fillRect(rightX + 8, tempY + 56, rightW - 16, 12, BG_COLOR);
  tft.setCursor(rightX + 8, tempY + 58);
  tft.print("Coolant");
}

static inline int16_t screen1MapY(int16_t y) {
  const float topGap = 6.0f + (float)DASH_TOP_OFFSET_PX;
  const float bottomGap = 6.0f;
  const float scaleY = (320.0f - topGap - bottomGap) / 320.0f;
  return (int16_t)roundf(topGap + (y * scaleY));
}

static inline int16_t screen1ScaleYLen(int16_t length) {
  const float topGap = 6.0f + (float)DASH_TOP_OFFSET_PX;
  const float bottomGap = 6.0f;
  const float scaleY = (320.0f - topGap - bottomGap) / 320.0f;
  return (int16_t)roundf(length * scaleY);
}

static inline int16_t screen2MapY(int16_t y) {
  const float topGap = 6.0f + (float)DASH_TOP_OFFSET_PX;
  const float bottomGap = 6.0f;
  const float scaleY = (320.0f - topGap - bottomGap) / 320.0f;
  return (int16_t)roundf(topGap + (y * scaleY));
}

static inline int16_t screen2ScaleYLen(int16_t length) {
  const float topGap = 6.0f + (float)DASH_TOP_OFFSET_PX;
  const float bottomGap = 6.0f;
  const float scaleY = (320.0f - topGap - bottomGap) / 320.0f;
  return (int16_t)roundf(length * scaleY);
}

// [BEGIN lopaka generated]
void drawScreen_1(void) {
  tft.fillScreen(BG_COLOR);
}
// [END lopaka generated]

void drawScreen_2(void) {
  tft2.fillScreen(BG_COLOR);
  // The speed gauge uses a custom 270° arc rather than a full circle so there is no
  // unwanted white border along the lower edge of the dial.

  tft2.setTextColor(DISPLAY_WHITE, BG_COLOR);
  tft2.setTextWrap(false);
  tft2.setTextSize(1);
  tft2.setCursor(317, screen2MapY(232));
  tft2.print("Odometer");
  tft2.setCursor(317, screen2MapY(125));
  tft2.print("MPH");
}

void drawOdometerPanel(float miles, bool fullRedraw) {
  const uint16_t panelX = 316;
  const uint16_t panelY = (uint16_t)screen2MapY(242);
  const uint16_t panelW = 160;
  const uint16_t panelH = (uint16_t)screen2ScaleYLen(76);

  tft2.fillRect(panelX + 1, panelY + 1, panelW - 2, panelH - 2, BG_COLOR);

  tft2.setTextColor(ST77XX_WHITE, BG_COLOR);
  tft2.setTextSize(1);
  tft2.setCursor(panelX + 8, panelY + 8);

  char milesText[20];
  snprintf(milesText, sizeof(milesText), "%07.1f", miles);
  tft2.setTextColor(VALUE_COLOR, BG_COLOR);
  tft2.setTextSize(2);
  int16_t textX = panelX + 10;
  int16_t textY = panelY + 26;
  tft2.setCursor(textX, textY);
  tft2.print(milesText);

  tft2.setTextColor(ST77XX_WHITE, BG_COLOR);
  tft2.setTextSize(1);
  tft2.setCursor(panelX + 124, panelY + 56);
  tft2.print("MI");
}

void drawSpeedDial(float mph, bool fullRedraw = false) {
  const int16_t cx = 170;
  const int16_t cy = screen2MapY(166);
  const int16_t dialRadius = screen2ScaleYLen(128);
  const int16_t needleRadius = screen2ScaleYLen(95);
  const int16_t labelRadius = dialRadius + 20;
  // 270° centered sweep: upper-left through bottom to lower-right.
  const float startDeg = 135.0f;
  const float endDeg = 405.0f;

  const int16_t speedBoxX = 316;
  const int16_t speedBoxY = screen2MapY(135);
  const int16_t speedBoxW = 159;
  const int16_t speedBoxH = screen2ScaleYLen(97);

  if (fullRedraw) {
    tft2.fillCircle(cx, cy, dialRadius + 12, BG_COLOR);

    for (int mphTick = 0; mphTick <= 100; mphTick += 5) {
      float ratio = (float)mphTick / 100.0f;
      float angleDeg = startDeg + ((endDeg - startDeg) * ratio);
      float angleRad = angleDeg * 0.0174532925f;

      int16_t outerX = cx + (int16_t)(dialRadius * cosf(angleRad));
      int16_t outerY = cy + (int16_t)(dialRadius * sinf(angleRad));
      int16_t innerRadius = (mphTick % 20 == 0) ? (dialRadius - 18) : (dialRadius - 12);
      int16_t innerX = cx + (int16_t)(innerRadius * cosf(angleRad));
      int16_t innerY = cy + (int16_t)(innerRadius * sinf(angleRad));
      tft2.drawLine(innerX, innerY, outerX, outerY, DISPLAY_GREEN);

      if (mphTick % 10 == 0) {
        int16_t labelX = cx + (int16_t)(labelRadius * cosf(angleRad));
        int16_t labelY = cy + (int16_t)(labelRadius * sinf(angleRad));
        char tickLabel[6];
        snprintf(tickLabel, sizeof(tickLabel), "%d", mphTick);

        tft2.setTextColor(DISPLAY_GREEN, BG_COLOR);
        tft2.setTextSize(2);
        tft2.setCursor(labelX - ((int16_t)strlen(tickLabel) * 6), labelY - 6);
        tft2.print(tickLabel);
      }
    }

    tft2.fillRect(speedBoxX + 1, speedBoxY + 1, speedBoxW - 2, speedBoxH - 2, BG_COLOR);
    tft2.setTextColor(DISPLAY_GREEN, BG_COLOR);
    tft2.setTextSize(1);
    tft2.setCursor(cx - 10, cy + dialRadius - 12);
    tft2.print("MPH");

    speedNeedlePrevX = -1;
    speedNeedlePrevY = -1;
  }

  if (speedNeedlePrevX >= 0 && speedNeedlePrevY >= 0) {
    int16_t prevDx = speedNeedlePrevX - cx;
    int16_t prevDy = speedNeedlePrevY - cy;
    float prevLen = sqrtf((float)(prevDx * prevDx + prevDy * prevDy));
    int16_t prevPerpX = (prevLen > 0.1f) ? (int16_t)(-prevDy * 2.2f / prevLen) : 0;
    int16_t prevPerpY = (prevLen > 0.1f) ? (int16_t)(prevDx * 2.2f / prevLen) : 0;

    drawTaperedNeedle(tft2, cx, cy, speedNeedlePrevX, speedNeedlePrevY, BG_COLOR);
  }

  float clampedMph = constrain(mph, 0.0f, SPEED_DIAL_MAX_MPH);
  float angleDeg = startDeg + ((endDeg - startDeg) * (clampedMph / SPEED_DIAL_MAX_MPH));
  float angleRad = angleDeg * 0.0174532925f;

  int16_t needleX = cx + (int16_t)((needleRadius - 8) * cosf(angleRad));
  int16_t needleY = cy + (int16_t)((needleRadius - 8) * sinf(angleRad));

  int16_t dx = needleX - cx;
  int16_t dy = needleY - cy;
  float needleLen = sqrtf((float)(dx * dx + dy * dy));
  int16_t perpX = (needleLen > 0.1f) ? (int16_t)(-dy * 2.2f / needleLen) : 0;
  int16_t perpY = (needleLen > 0.1f) ? (int16_t)(dx * 2.2f / needleLen) : 0;

  drawTaperedNeedle(tft2, cx, cy, needleX, needleY, NEEDLE_COLOR);

  tft2.fillCircle(cx, cy, 5, NEEDLE_COLOR);
  tft2.drawCircle(cx, cy, 5, DISPLAY_GRAY);

  speedNeedlePrevX = needleX;
  speedNeedlePrevY = needleY;

  int speedRounded = (int)roundf(mph);
  char speedText[12];
  snprintf(speedText, sizeof(speedText), "%d", speedRounded);
  tft2.setTextColor(DISPLAY_GREEN, BG_COLOR);
  tft2.setTextSize(5);
  tft2.fillRect(speedBoxX + 4, speedBoxY + 16, speedBoxW - 8, 52, BG_COLOR);
  int16_t speedTextX = speedBoxX + (speedBoxW / 2) - ((int16_t)strlen(speedText) * 15);
  tft2.setCursor(speedTextX, speedBoxY + 18);
  tft2.print(speedText);

  tft2.setTextColor(ST77XX_WHITE, BG_COLOR);
  tft2.setTextSize(1);
  tft2.fillRect(speedBoxX + 36, speedBoxY + 70, 84, 16, BG_COLOR);
  tft2.setCursor(speedBoxX + 60, speedBoxY + 74);
  tft2.print("MPH");
}

void drawFuelVoltageReadout(float volts) {
  const int16_t cx = 94;
  const int16_t cy = screen1MapY(SCREEN1_GAUGE_TOP_CENTER_Y);
  const int16_t radius = screen1ScaleYLen(SCREEN1_GAUGE_RADIUS);
  const int16_t pivotY = cy + screen1ScaleYLen(6);
  const float leftAng = 180.0f * 0.0174532925f;
  const int16_t leftX = cx + (int16_t)((radius - 6) * cosf(leftAng));
  const int16_t leftY = pivotY + (int16_t)((radius - 6) * sinf(leftAng));

  char voltText[16];
  snprintf(voltText, sizeof(voltText), "%.2fV", volts);

  // Position the voltage readout directly below the E mark on the fuel arc.
  const int16_t textX = leftX + 8;
  const int16_t textY = leftY + 30;
  tft.fillRect(textX - 2, textY - 2, 66, 14, BG_COLOR);
  tft.setTextColor(DISPLAY_WHITE, BG_COLOR);
  tft.setTextSize(1);
  tft.setCursor(textX, textY);
  tft.print(voltText);
}

void drawLowFuelIndicator(bool visible) {
  const int16_t warningX = 200;
  const int16_t warningY = screen1MapY(120);
  const int16_t warningW = 90;
  const int16_t warningH = 54;
  const int16_t iconX = warningX + 4;
  const int16_t iconY = warningY + 12;

  tft.fillRect(warningX, warningY, warningW, warningH, BG_COLOR);
  if (!visible) {
    return;
  }

  for (int16_t o = 0; o <= 1; o++) {
    tft.drawRoundRect(iconX - o, iconY - o, 20 + 2 * o, 30 + 2 * o, 3, WARNING_COLOR);
    tft.drawRect(iconX + 5 - o, iconY + 6 - o, 10 + 2 * o, 8 + 2 * o, WARNING_COLOR);
  }
  for (int16_t o = -1; o <= 1; o++) {
    tft.drawLine(iconX + 20, iconY + 5 + o, iconX + 26, iconY + 5 + o, WARNING_COLOR);
    tft.drawLine(iconX + 26 + o, iconY + 5, iconX + 26 + o, iconY + 20, WARNING_COLOR);
    tft.drawLine(iconX + 26, iconY + 20 + o, iconX + 22, iconY + 20 + o, WARNING_COLOR);
    tft.drawLine(iconX + 5, iconY + 30 + o, iconX + 15, iconY + 30 + o, WARNING_COLOR);
  }

  tft.setTextColor(WARNING_COLOR, BG_COLOR);
  tft.setTextSize(2);
  tft.setCursor(warningX + 38, warningY + 21);
  tft.print("LOW");
}

void drawGaugeWarningIndicator(int16_t warningY, const char* label, bool visible) {
  const int16_t warningX = 198;
  const int16_t warningW = 84;
  const int16_t warningH = 54;
  const int16_t iconX = warningX + 2;
  const int16_t iconY = warningY + 12;

  tft.fillRect(warningX, warningY, warningW, warningH, BG_COLOR);
  if (!visible) {
    return;
  }

  tft.fillTriangle(iconX + 12, iconY, iconX, iconY + 28,
                   iconX + 24, iconY + 28, WARNING_COLOR);
  for (int16_t o = -1; o <= 1; o++) {
    tft.drawLine(iconX + 12 + o, iconY + 9, iconX + 12 + o, iconY + 19, BG_COLOR);
  }
  tft.fillCircle(iconX + 12, iconY + 24, 2, BG_COLOR);

  tft.setTextColor(WARNING_COLOR, BG_COLOR);
  tft.setTextSize(2);
  tft.setCursor(warningX + 30, warningY + 21);
  tft.print(label);
}

void drawGaugeLeftLabel(int16_t leftX, int16_t leftY, const char* label) {
  // Place labels below the small gauge readouts and outside the needle sweep.
  const int16_t textX = (leftX < 8) ? 8 : (leftX - 4);
  const int16_t textY = leftY + 42;
  int16_t labelWidth = ((int16_t)strlen(label) * 6) + 6;
  if (labelWidth < 36) {
    labelWidth = 36;
  }

  tft.fillRect(textX - 2, textY - 2, labelWidth, 12, BG_COLOR);
  tft.setTextColor(DISPLAY_WHITE, BG_COLOR);
  tft.setTextSize(1);
  tft.setCursor(textX, textY);
  tft.print(label);
}

void drawBatteryDial(float volts, bool fullRedraw) {
  const int16_t cx = 94;
  const int16_t cy = screen1MapY(SCREEN1_GAUGE_BOTTOM_CENTER_Y);
  const int16_t radius = screen1ScaleYLen(SCREEN1_GAUGE_RADIUS);
  const int16_t pivotY = cy + screen1ScaleYLen(6);
  const float startDeg = SCREEN1_GAUGE_START_DEG;
  const float endDeg = SCREEN1_GAUGE_END_DEG;
  const float leftAng = startDeg * 0.0174532925f;

  const int16_t battBoxX = 17;
  const int16_t battBoxY = screen1MapY(244);
  const int16_t battBoxW = 152;
  const int16_t battBoxH = screen1ScaleYLen(97);

  const int16_t leftX = cx + (int16_t)((radius - 6) * cosf(leftAng));
  const int16_t leftY = pivotY + (int16_t)((radius - 6) * sinf(leftAng));

  if (fullRedraw) {
    tft.fillCircle(cx, cy, radius - 3, BG_COLOR);
    tft.fillRect(battBoxX + 1, battBoxY + 1, battBoxW - 2, battBoxH - 2, BG_COLOR);

    for (int i = 0; i <= 4; i++) {
      float ratio = i / 4.0f;
      float angleDeg = startDeg + ((endDeg - startDeg) * ratio);
      float angleRad = angleDeg * 0.0174532925f;

      int16_t outerX = cx + (int16_t)((radius - 2) * cosf(angleRad));
      int16_t outerY = pivotY + (int16_t)((radius - 2) * sinf(angleRad));
      int16_t innerX = cx + (int16_t)((radius - 16) * cosf(angleRad));
      int16_t innerY = pivotY + (int16_t)((radius - 16) * sinf(angleRad));
      for (int16_t o = -1; o <= 2; o++) {
        tft.drawLine(innerX + o, innerY, outerX + o, outerY, DISPLAY_WHITE);
      }
    }

    const float rightAng = endDeg * 0.0174532925f;
    int16_t rightX = cx + (int16_t)((radius - 6) * cosf(rightAng));
    int16_t rightY = pivotY + (int16_t)((radius - 6) * sinf(rightAng));

    const float markerInsetDeg = 14.0f;
    const float leftMarkerAng = (startDeg + markerInsetDeg) * 0.0174532925f;
    const float rightMarkerAng = (endDeg - markerInsetDeg) * 0.0174532925f;
    const float markerHalfSpanDeg = 7.5f;
    const float markerInnerR = radius - 16.0f;
    const float markerOuterR = radius - 3.0f;
    const float markerStepDeg = 0.05f;

    for (float a = (leftMarkerAng - (markerHalfSpanDeg * 0.0174532925f));
         a <= (leftMarkerAng + (markerHalfSpanDeg * 0.0174532925f));
         a += (markerStepDeg * 0.0174532925f)) {
      int16_t x0 = cx + (int16_t)roundf(markerInnerR * cosf(a));
      int16_t y0 = pivotY + (int16_t)roundf(markerInnerR * sinf(a));
      int16_t x1 = cx + (int16_t)roundf(markerOuterR * cosf(a));
      int16_t y1 = pivotY + (int16_t)roundf(markerOuterR * sinf(a));
      tft.drawLine(x0, y0, x1, y1, ST77XX_RED);

      float aMid = a + ((markerStepDeg * 0.5f) * 0.0174532925f);
      int16_t mx0 = cx + (int16_t)roundf(markerInnerR * cosf(aMid));
      int16_t my0 = pivotY + (int16_t)roundf(markerInnerR * sinf(aMid));
      int16_t mx1 = cx + (int16_t)roundf(markerOuterR * cosf(aMid));
      int16_t my1 = pivotY + (int16_t)roundf(markerOuterR * sinf(aMid));
      tft.drawLine(mx0, my0, mx1, my1, ST77XX_RED);
    }

    for (float a = (rightMarkerAng - (markerHalfSpanDeg * 0.0174532925f));
         a <= (rightMarkerAng + (markerHalfSpanDeg * 0.0174532925f));
         a += (markerStepDeg * 0.0174532925f)) {
      int16_t x0 = cx + (int16_t)roundf(markerInnerR * cosf(a));
      int16_t y0 = pivotY + (int16_t)roundf(markerInnerR * sinf(a));
      int16_t x1 = cx + (int16_t)roundf(markerOuterR * cosf(a));
      int16_t y1 = pivotY + (int16_t)roundf(markerOuterR * sinf(a));
      tft.drawLine(x0, y0, x1, y1, DISPLAY_BLUE);

      float aMid = a + ((markerStepDeg * 0.5f) * 0.0174532925f);
      int16_t mx0 = cx + (int16_t)roundf(markerInnerR * cosf(aMid));
      int16_t my0 = pivotY + (int16_t)roundf(markerInnerR * sinf(aMid));
      int16_t mx1 = cx + (int16_t)roundf(markerOuterR * cosf(aMid));
      int16_t my1 = pivotY + (int16_t)roundf(markerOuterR * sinf(aMid));
      tft.drawLine(mx0, my0, mx1, my1, DISPLAY_BLUE);
    }

    tft.setTextColor(DISPLAY_WHITE, BG_COLOR);
    tft.setTextSize(2);
    tft.setCursor(leftX + 8, leftY + 8);
    tft.print("L");
    tft.setCursor(rightX + 10, rightY + 8);
    tft.print("H");
    drawGaugeLeftLabel(leftX, leftY, "Battery");

    batteryNeedlePrevX = -1;
    batteryNeedlePrevY = -1;
  }

  if (batteryNeedlePrevX >= 0 && batteryNeedlePrevY >= 0) {
    int16_t prevDx = batteryNeedlePrevX - cx;
    int16_t prevDy = batteryNeedlePrevY - pivotY;
    float prevLen = sqrtf((float)(prevDx * prevDx + prevDy * prevDy));
    int16_t prevPerpX = (prevLen > 0.1f) ? (int16_t)(-prevDy * 1.5f / prevLen) : 0;
    int16_t prevPerpY = (prevLen > 0.1f) ? (int16_t)(prevDx * 1.5f / prevLen) : 0;

        drawTaperedNeedle(tft, cx, pivotY, batteryNeedlePrevX, batteryNeedlePrevY, BG_COLOR);
  }

  char battText[10];
  snprintf(battText, sizeof(battText), "%.1fV", volts);
  tft.setTextColor(DISPLAY_WHITE, BG_COLOR);
  tft.setTextSize(1);

  // Keep the readout below the pivot, outside the needle sweep.
  const int16_t textX = cx - 20;
  const int16_t textY = pivotY + 34;
  tft.fillRect(textX - 2, textY - 2, 48, 12, BG_COLOR);
  tft.setCursor(textX, textY);
  tft.print(battText);

  float clampedVolts = constrain(volts, BATTERY_DIAL_MIN_V, BATTERY_DIAL_MAX_V);
  float ratio = (clampedVolts - BATTERY_DIAL_MIN_V) / (BATTERY_DIAL_MAX_V - BATTERY_DIAL_MIN_V);
  float angleDeg = startDeg + ((endDeg - startDeg) * ratio);
  float angleRad = angleDeg * 0.0174532925f;

  int16_t needleX = cx + (int16_t)((radius - 24) * cosf(angleRad));
  int16_t needleY = pivotY + (int16_t)((radius - 24) * sinf(angleRad));

  int16_t dx = needleX - cx;
  int16_t dy = needleY - pivotY;
  float needleLen = sqrtf((float)(dx * dx + dy * dy));
  int16_t perpX = (needleLen > 0.1f) ? (int16_t)(-dy * 1.5f / needleLen) : 0;
  int16_t perpY = (needleLen > 0.1f) ? (int16_t)(dx * 1.5f / needleLen) : 0;

  drawTaperedNeedle(tft, cx, pivotY, needleX, needleY, NEEDLE_COLOR);

  tft.fillCircle(cx, pivotY, 10, NEEDLE_COLOR);
  tft.drawCircle(cx, pivotY, 10, DISPLAY_GRAY);

  batteryNeedlePrevX = needleX;
  batteryNeedlePrevY = needleY;
}

// Fuel dial gauge for the first display. Mirrors the speedometer dial's look
// (same sweep angles/needle style) and shrinks its top the same way the main
// dashboard does via DASH_TOP_OFFSET_PX.
void drawFuelDial(int percent, bool fullRedraw) {
  const int16_t cx = 94;
  const int16_t cy = screen1MapY(SCREEN1_GAUGE_TOP_CENTER_Y);
  const int16_t radius = screen1ScaleYLen(SCREEN1_GAUGE_RADIUS);
  const int16_t pivotY = cy + screen1ScaleYLen(6);
  const float startDeg = SCREEN1_GAUGE_START_DEG;
  const float endDeg = SCREEN1_GAUGE_END_DEG;

  const int16_t fuelBoxX = 17;
  const int16_t fuelBoxY = screen1MapY(218);
  const int16_t fuelBoxW = 152;
  const int16_t fuelBoxH = screen1ScaleYLen(97);

  if (fullRedraw) {
    // Match coolant dial style: dark face, no bright outer rings.
    tft.fillCircle(cx, cy, radius - 3, BG_COLOR);
    tft.fillRect(fuelBoxX + 1, fuelBoxY + 1, fuelBoxW - 2, fuelBoxH - 2, BG_COLOR);

    for (int i = 0; i <= 4; i++) {
      float ratio = i / 4.0f;
      float angleDeg = startDeg + ((endDeg - startDeg) * ratio);
      float angleRad = angleDeg * 0.0174532925f;

      int16_t outerX = cx + (int16_t)((radius - 2) * cosf(angleRad));
      int16_t outerY = pivotY + (int16_t)((radius - 2) * sinf(angleRad));
      int16_t innerX = cx + (int16_t)((radius - 16) * cosf(angleRad));
      int16_t innerY = pivotY + (int16_t)((radius - 16) * sinf(angleRad));
      for (int16_t o = -1; o <= 2; o++) {
        tft.drawLine(innerX + o, innerY, outerX + o, outerY, DISPLAY_WHITE);
      }
    }

    const float leftAng = startDeg * 0.0174532925f;
    const float rightAng = endDeg * 0.0174532925f;
    int16_t leftX = cx + (int16_t)((radius - 6) * cosf(leftAng));
    int16_t leftY = pivotY + (int16_t)((radius - 6) * sinf(leftAng));
    int16_t rightX = cx + (int16_t)((radius - 6) * cosf(rightAng));
    int16_t rightY = pivotY + (int16_t)((radius - 6) * sinf(rightAng));

    const float markerInsetDeg = 14.0f;
    const float leftMarkerAng = (startDeg + markerInsetDeg) * 0.0174532925f;
    const float rightMarkerAng = (endDeg - markerInsetDeg) * 0.0174532925f;

    const float markerHalfSpanDeg = 7.5f;
    const float markerInnerR = radius - 16.0f;
    const float markerOuterR = radius - 3.0f;
    const float markerStepDeg = 0.05f;

    for (float a = (leftMarkerAng - (markerHalfSpanDeg * 0.0174532925f));
         a <= (leftMarkerAng + (markerHalfSpanDeg * 0.0174532925f));
         a += (markerStepDeg * 0.0174532925f)) {
      int16_t x0 = cx + (int16_t)roundf(markerInnerR * cosf(a));
      int16_t y0 = pivotY + (int16_t)roundf(markerInnerR * sinf(a));
      int16_t x1 = cx + (int16_t)roundf(markerOuterR * cosf(a));
      int16_t y1 = pivotY + (int16_t)roundf(markerOuterR * sinf(a));
      tft.drawLine(x0, y0, x1, y1, ST77XX_RED);

      float aMid = a + ((markerStepDeg * 0.5f) * 0.0174532925f);
      int16_t mx0 = cx + (int16_t)roundf(markerInnerR * cosf(aMid));
      int16_t my0 = pivotY + (int16_t)roundf(markerInnerR * sinf(aMid));
      int16_t mx1 = cx + (int16_t)roundf(markerOuterR * cosf(aMid));
      int16_t my1 = pivotY + (int16_t)roundf(markerOuterR * sinf(aMid));
      tft.drawLine(mx0, my0, mx1, my1, ST77XX_RED);
    }

    tft.setTextColor(DISPLAY_WHITE, BG_COLOR);
    tft.setTextSize(2);
    tft.setCursor(leftX + 8, leftY + 8);
    tft.print("E");
    tft.setCursor(rightX + 10, rightY + 8);
    tft.print("F");
    drawGaugeLeftLabel(leftX, leftY, "Fuel");

    fuelNeedlePrevX = -1;
    fuelNeedlePrevY = -1;
  }

  if (fuelNeedlePrevX >= 0 && fuelNeedlePrevY >= 0) {
    // Erase the previous needle with the same geometry used for drawing.
    int16_t prevDx = fuelNeedlePrevX - cx;
    int16_t prevDy = fuelNeedlePrevY - pivotY;
    float prevLen = sqrtf((float)(prevDx * prevDx + prevDy * prevDy));
    int16_t prevPerpX = (prevLen > 0.1f) ? (int16_t)(-prevDy * 1.5f / prevLen) : 0;
    int16_t prevPerpY = (prevLen > 0.1f) ? (int16_t)(prevDx * 1.5f / prevLen) : 0;

        drawTaperedNeedle(tft, cx, pivotY, fuelNeedlePrevX, fuelNeedlePrevY, BG_COLOR);
  }

  float clampedPercent = constrain((float)percent, 0.0f, 100.0f);
  float angleDeg = startDeg + ((endDeg - startDeg) * (clampedPercent / 100.0f));
  float angleRad = angleDeg * 0.0174532925f;

  int16_t needleX = cx + (int16_t)((radius - 24) * cosf(angleRad));
  int16_t needleY = pivotY + (int16_t)((radius - 24) * sinf(angleRad));

  // Draw coolant-style white needle.
  int16_t dx = needleX - cx;
  int16_t dy = needleY - pivotY;
  float needleLen = sqrtf((float)(dx * dx + dy * dy));
  int16_t perpX = (needleLen > 0.1f) ? (int16_t)(-dy * 1.5f / needleLen) : 0;
  int16_t perpY = (needleLen > 0.1f) ? (int16_t)(dx * 1.5f / needleLen) : 0;

  drawTaperedNeedle(tft, cx, pivotY, needleX, needleY, NEEDLE_COLOR);

  tft.fillCircle(cx, pivotY, 10, NEEDLE_COLOR);
  tft.drawCircle(cx, pivotY, 10, DISPLAY_GRAY);

  fuelNeedlePrevX = needleX;
  fuelNeedlePrevY = needleY;

  const int16_t fuelReadoutY = pivotY + radius + 8;
  tft.fillRect(cx - 46, fuelReadoutY - 2, 92, 18, BG_COLOR);

  drawFuelVoltageReadout(fuelVoltage);
  drawLowFuelIndicator(percent < 25);
}

void drawCoolantDial(float tempF, bool fullRedraw) {
  const int16_t cx = 384;
  const int16_t cy = screen1MapY(SCREEN1_GAUGE_TOP_CENTER_Y);
  const int16_t radius = screen1ScaleYLen(SCREEN1_GAUGE_RADIUS);
  const int16_t pivotY = cy + screen1ScaleYLen(6);
  const float startDeg = SCREEN1_GAUGE_START_DEG;
  const float endDeg = SCREEN1_GAUGE_END_DEG;

  const int16_t tempBoxX = 307;
  const int16_t tempBoxY = screen1MapY(217);
  const int16_t tempBoxW = 152;
  const int16_t tempBoxH = screen1ScaleYLen(97);

  if (fullRedraw) {
    // Keep this gauge on the dark dashboard background and fit it inside the dial circle.
    tft.fillCircle(cx, cy, radius - 3, BG_COLOR);
    tft.fillRect(tempBoxX + 1, tempBoxY + 1, tempBoxW - 2, tempBoxH - 2, BG_COLOR);

    for (int i = 0; i <= 4; i++) {
      float ratio = i / 4.0f;
      float angleDeg = startDeg + ((endDeg - startDeg) * ratio);
      float angleRad = angleDeg * 0.0174532925f;

      int16_t outerX = cx + (int16_t)((radius - 2) * cosf(angleRad));
      int16_t outerY = pivotY + (int16_t)((radius - 2) * sinf(angleRad));
      int16_t innerX = cx + (int16_t)((radius - 16) * cosf(angleRad));
      int16_t innerY = pivotY + (int16_t)((radius - 16) * sinf(angleRad));
      for (int16_t o = -1; o <= 2; o++) {
        tft.drawLine(innerX + o, innerY, outerX + o, outerY, DISPLAY_WHITE);
      }
    }

    const float leftAng = startDeg * 0.0174532925f;
    const float rightAng = endDeg * 0.0174532925f;
    int16_t leftX = cx + (int16_t)((radius - 6) * cosf(leftAng));
    int16_t leftY = pivotY + (int16_t)((radius - 6) * sinf(leftAng));
    int16_t rightX = cx + (int16_t)((radius - 6) * cosf(rightAng));
    int16_t rightY = pivotY + (int16_t)((radius - 6) * sinf(rightAng));

    // Place color markers between the endpoint ticks and center ticks.
    const float markerInsetDeg = 14.0f;
    const float leftMarkerAng = (startDeg + markerInsetDeg) * 0.0174532925f;
    const float rightMarkerAng = (endDeg - markerInsetDeg) * 0.0174532925f;
    int16_t leftMarkerX = cx + (int16_t)((radius - 6) * cosf(leftMarkerAng));
    int16_t leftMarkerY = pivotY + (int16_t)((radius - 6) * sinf(leftMarkerAng));
    int16_t rightMarkerX = cx + (int16_t)((radius - 6) * cosf(rightMarkerAng));
    int16_t rightMarkerY = pivotY + (int16_t)((radius - 6) * sinf(rightMarkerAng));

    // Draw short arc blocks so the markers conform to the gauge curvature.
    const float markerHalfSpanDeg = 7.5f;
    const float markerInnerR = radius - 16.0f;
    const float markerOuterR = radius - 3.0f;
    const float markerStepDeg = 0.05f;

    for (float a = (leftMarkerAng - (markerHalfSpanDeg * 0.0174532925f));
         a <= (leftMarkerAng + (markerHalfSpanDeg * 0.0174532925f));
         a += (markerStepDeg * 0.0174532925f)) {
      int16_t x0 = cx + (int16_t)roundf(markerInnerR * cosf(a));
      int16_t y0 = pivotY + (int16_t)roundf(markerInnerR * sinf(a));
      int16_t x1 = cx + (int16_t)roundf(markerOuterR * cosf(a));
      int16_t y1 = pivotY + (int16_t)roundf(markerOuterR * sinf(a));
      tft.drawLine(x0, y0, x1, y1, ST77XX_RED);

      float aMid = a + ((markerStepDeg * 0.5f) * 0.0174532925f);
      int16_t mx0 = cx + (int16_t)roundf(markerInnerR * cosf(aMid));
      int16_t my0 = pivotY + (int16_t)roundf(markerInnerR * sinf(aMid));
      int16_t mx1 = cx + (int16_t)roundf(markerOuterR * cosf(aMid));
      int16_t my1 = pivotY + (int16_t)roundf(markerOuterR * sinf(aMid));
      tft.drawLine(mx0, my0, mx1, my1, ST77XX_RED);
    }

    for (float a = (rightMarkerAng - (markerHalfSpanDeg * 0.0174532925f));
         a <= (rightMarkerAng + (markerHalfSpanDeg * 0.0174532925f));
         a += (markerStepDeg * 0.0174532925f)) {
      int16_t x0 = cx + (int16_t)roundf(markerInnerR * cosf(a));
      int16_t y0 = pivotY + (int16_t)roundf(markerInnerR * sinf(a));
      int16_t x1 = cx + (int16_t)roundf(markerOuterR * cosf(a));
      int16_t y1 = pivotY + (int16_t)roundf(markerOuterR * sinf(a));
      tft.drawLine(x0, y0, x1, y1, DISPLAY_BLUE);

      float aMid = a + ((markerStepDeg * 0.5f) * 0.0174532925f);
      int16_t mx0 = cx + (int16_t)roundf(markerInnerR * cosf(aMid));
      int16_t my0 = pivotY + (int16_t)roundf(markerInnerR * sinf(aMid));
      int16_t mx1 = cx + (int16_t)roundf(markerOuterR * cosf(aMid));
      int16_t my1 = pivotY + (int16_t)roundf(markerOuterR * sinf(aMid));
      tft.drawLine(mx0, my0, mx1, my1, DISPLAY_BLUE);
    }

    tft.setTextColor(DISPLAY_WHITE, BG_COLOR);
    tft.setTextSize(2);
    tft.setCursor(leftX + 8, leftY + 8);
    tft.print("C");
    tft.setCursor(rightX + 10, rightY + 8);
    tft.print("H");
    drawGaugeLeftLabel(leftX, leftY, "Coolant");

    coolantNeedlePrevX = -1;
    coolantNeedlePrevY = -1;
  }

  if (coolantNeedlePrevX >= 0 && coolantNeedlePrevY >= 0) {
    int16_t prevDx = coolantNeedlePrevX - cx;
    int16_t prevDy = coolantNeedlePrevY - pivotY;
    float prevLen = sqrtf((float)(prevDx * prevDx + prevDy * prevDy));
    int16_t prevPerpX = (prevLen > 0.1f) ? (int16_t)(-prevDy * 1.5f / prevLen) : 0;
    int16_t prevPerpY = (prevLen > 0.1f) ? (int16_t)(prevDx * 1.5f / prevLen) : 0;

    drawTaperedNeedle(tft, cx, pivotY, coolantNeedlePrevX, coolantNeedlePrevY, BG_COLOR);
  }

  float clampedTemp = constrain(tempF, COOLANT_DIAL_MIN_F, COOLANT_DIAL_MAX_F);
  float ratio = (clampedTemp - COOLANT_DIAL_MIN_F) / (COOLANT_DIAL_MAX_F - COOLANT_DIAL_MIN_F);
  float angleDeg = startDeg + ((endDeg - startDeg) * ratio);
  float angleRad = angleDeg * 0.0174532925f;

  int16_t needleX = cx + (int16_t)((radius - 24) * cosf(angleRad));
  int16_t needleY = pivotY + (int16_t)((radius - 24) * sinf(angleRad));

  int16_t dx = needleX - cx;
  int16_t dy = needleY - pivotY;
  float needleLen = sqrtf((float)(dx * dx + dy * dy));
  int16_t perpX = (needleLen > 0.1f) ? (int16_t)(-dy * 1.5f / needleLen) : 0;
  int16_t perpY = (needleLen > 0.1f) ? (int16_t)(dx * 1.5f / needleLen) : 0;

  drawTaperedNeedle(tft, cx, pivotY, needleX, needleY, NEEDLE_COLOR);

  tft.fillCircle(cx, pivotY, 10, NEEDLE_COLOR);
  tft.drawCircle(cx, pivotY, 10, DISPLAY_GRAY);

  coolantNeedlePrevX = needleX;
  coolantNeedlePrevY = needleY;

  const int16_t tempReadoutY = pivotY + radius + 8;
  tft.fillRect(cx - 46, tempReadoutY - 2, 92, 18, BG_COLOR);
  drawGaugeWarningIndicator(screen1MapY(55), "TEMP", tempF < COOLANT_LOW_WARNING_F);
}

void drawOilPsiDial(float psi, bool fullRedraw) {
  const int16_t cx = 384;
  const int16_t cy = screen1MapY(SCREEN1_GAUGE_BOTTOM_CENTER_Y);
  const int16_t radius = screen1ScaleYLen(SCREEN1_GAUGE_RADIUS);
  const int16_t pivotY = cy + screen1ScaleYLen(6);
  const float startDeg = SCREEN1_GAUGE_START_DEG;
  const float endDeg = SCREEN1_GAUGE_END_DEG;

  const int16_t oilBoxX = 307;
  const int16_t oilBoxY = screen1MapY(244);
  const int16_t oilBoxW = 152;
  const int16_t oilBoxH = screen1ScaleYLen(97);

  if (fullRedraw) {
    tft.fillCircle(cx, cy, radius - 3, BG_COLOR);
    tft.fillRect(oilBoxX + 1, oilBoxY + 1, oilBoxW - 2, oilBoxH - 2, BG_COLOR);

    for (int i = 0; i <= 4; i++) {
      float ratio = i / 4.0f;
      float angleDeg = startDeg + ((endDeg - startDeg) * ratio);
      float angleRad = angleDeg * 0.0174532925f;

      int16_t outerX = cx + (int16_t)((radius - 2) * cosf(angleRad));
      int16_t outerY = pivotY + (int16_t)((radius - 2) * sinf(angleRad));
      int16_t innerX = cx + (int16_t)((radius - 16) * cosf(angleRad));
      int16_t innerY = pivotY + (int16_t)((radius - 16) * sinf(angleRad));
      for (int16_t o = -1; o <= 2; o++) {
        tft.drawLine(innerX + o, innerY, outerX + o, outerY, DISPLAY_WHITE);
      }
    }

    const float leftAng = startDeg * 0.0174532925f;
    const float rightAng = endDeg * 0.0174532925f;
    int16_t leftX = cx + (int16_t)((radius - 6) * cosf(leftAng));
    int16_t leftY = pivotY + (int16_t)((radius - 6) * sinf(leftAng));
    int16_t rightX = cx + (int16_t)((radius - 6) * cosf(rightAng));
    int16_t rightY = pivotY + (int16_t)((radius - 6) * sinf(rightAng));

    const float markerInsetDeg = 14.0f;
    const float leftMarkerAng = (startDeg + markerInsetDeg) * 0.0174532925f;
    const float rightMarkerAng = (endDeg - markerInsetDeg) * 0.0174532925f;
    const float markerHalfSpanDeg = 7.5f;
    const float markerInnerR = radius - 16.0f;
    const float markerOuterR = radius - 3.0f;
    const float markerStepDeg = 0.05f;

    for (float a = (leftMarkerAng - (markerHalfSpanDeg * 0.0174532925f));
         a <= (leftMarkerAng + (markerHalfSpanDeg * 0.0174532925f));
         a += (markerStepDeg * 0.0174532925f)) {
      int16_t x0 = cx + (int16_t)roundf(markerInnerR * cosf(a));
      int16_t y0 = pivotY + (int16_t)roundf(markerInnerR * sinf(a));
      int16_t x1 = cx + (int16_t)roundf(markerOuterR * cosf(a));
      int16_t y1 = pivotY + (int16_t)roundf(markerOuterR * sinf(a));
      tft.drawLine(x0, y0, x1, y1, ST77XX_RED);

      float aMid = a + ((markerStepDeg * 0.5f) * 0.0174532925f);
      int16_t mx0 = cx + (int16_t)roundf(markerInnerR * cosf(aMid));
      int16_t my0 = pivotY + (int16_t)roundf(markerInnerR * sinf(aMid));
      int16_t mx1 = cx + (int16_t)roundf(markerOuterR * cosf(aMid));
      int16_t my1 = pivotY + (int16_t)roundf(markerOuterR * sinf(aMid));
      tft.drawLine(mx0, my0, mx1, my1, ST77XX_RED);

    }

    for (float a = (rightMarkerAng - (markerHalfSpanDeg * 0.0174532925f));
         a <= (rightMarkerAng + (markerHalfSpanDeg * 0.0174532925f));
         a += (markerStepDeg * 0.0174532925f)) {
      int16_t x0 = cx + (int16_t)roundf(markerInnerR * cosf(a));
      int16_t y0 = pivotY + (int16_t)roundf(markerInnerR * sinf(a));
      int16_t x1 = cx + (int16_t)roundf(markerOuterR * cosf(a));
      int16_t y1 = pivotY + (int16_t)roundf(markerOuterR * sinf(a));
      tft.drawLine(x0, y0, x1, y1, DISPLAY_BLUE);

      float aMid = a + ((markerStepDeg * 0.5f) * 0.0174532925f);
      int16_t mx0 = cx + (int16_t)roundf(markerInnerR * cosf(aMid));
      int16_t my0 = pivotY + (int16_t)roundf(markerInnerR * sinf(aMid));
      int16_t mx1 = cx + (int16_t)roundf(markerOuterR * cosf(aMid));
      int16_t my1 = pivotY + (int16_t)roundf(markerOuterR * sinf(aMid));
      tft.drawLine(mx0, my0, mx1, my1, DISPLAY_BLUE);

    }

    tft.setTextColor(DISPLAY_WHITE, BG_COLOR);
    tft.setTextSize(2);
    tft.setCursor(leftX + 8, leftY + 8);
    tft.print("L");
    tft.setCursor(rightX + 10, rightY + 8);
    tft.print("H");
    drawGaugeLeftLabel(leftX, leftY, "Oil PSI");

    oilNeedlePrevX = -1;
    oilNeedlePrevY = -1;
  }

  if (oilNeedlePrevX >= 0 && oilNeedlePrevY >= 0) {
    int16_t prevDx = oilNeedlePrevX - cx;
    int16_t prevDy = oilNeedlePrevY - pivotY;
    float prevLen = sqrtf((float)(prevDx * prevDx + prevDy * prevDy));
    int16_t prevPerpX = (prevLen > 0.1f) ? (int16_t)(-prevDy * 1.5f / prevLen) : 0;
    int16_t prevPerpY = (prevLen > 0.1f) ? (int16_t)(prevDx * 1.5f / prevLen) : 0;

        drawTaperedNeedle(tft, cx, pivotY, oilNeedlePrevX, oilNeedlePrevY, BG_COLOR);
  }

  float clampedPsi = constrain(psi, OIL_PSI_MIN, OIL_PSI_MAX);
  float ratio = (clampedPsi - OIL_PSI_MIN) / (OIL_PSI_MAX - OIL_PSI_MIN);
  float angleDeg = startDeg + ((endDeg - startDeg) * ratio);
  float angleRad = angleDeg * 0.0174532925f;

  int16_t needleX = cx + (int16_t)((radius - 24) * cosf(angleRad));
  int16_t needleY = pivotY + (int16_t)((radius - 24) * sinf(angleRad));

  int16_t dx = needleX - cx;
  int16_t dy = needleY - pivotY;
  float needleLen = sqrtf((float)(dx * dx + dy * dy));
  int16_t perpX = (needleLen > 0.1f) ? (int16_t)(-dy * 1.5f / needleLen) : 0;
  int16_t perpY = (needleLen > 0.1f) ? (int16_t)(dx * 1.5f / needleLen) : 0;

  drawTaperedNeedle(tft, cx, pivotY, needleX, needleY, NEEDLE_COLOR);

  tft.fillCircle(cx, pivotY, 10, NEEDLE_COLOR);
  tft.drawCircle(cx, pivotY, 10, DISPLAY_GRAY);

  oilNeedlePrevX = needleX;
  oilNeedlePrevY = needleY;

  // Clear area where the oil readout text used to be.
  const int16_t readoutY = cy + radius + 8;
  tft.fillRect(cx - 46, readoutY - 2, 92, 28, BG_COLOR);
  drawGaugeWarningIndicator(screen1MapY(180), "OIL", psi < OIL_LOW_WARNING_PSI);
}

void drawDashboard() {
  fuelPercent = readFuelPercent();
  // Placeholder until a dedicated battery sense input is wired.
  batteryVoltage = 12.0f + (fuelVoltage * (4.0f / 3.3f));

  drawScreen_2();

  drawSpeedDial(speedMph, true);
  drawOdometerPanel(odometerMiles, true);
}

static inline bool valuesChanged(float a, float b, float epsilon) {
  return fabsf(a - b) > epsilon;
}

void renderDashboardFrame() {
  int localFuelPercent = fuelPercent;
  float localFuelVoltage = fuelVoltage;
  float localSpeedMph = speedMph;
  float localOdometerMiles = odometerMiles;
  float localBatteryVoltage = batteryVoltage;
  float localCoolantTempF = coolantTempF;
  float localOilPsi = oilPsi;

  if (lastRenderedFuelPercent != localFuelPercent) {
    drawFuelDial(localFuelPercent, false);
    lastRenderedFuelPercent = localFuelPercent;
  }

  if (valuesChanged(localCoolantTempF, lastRenderedCoolantTempF, 0.5f)) {
    drawCoolantDial(localCoolantTempF, false);
    lastRenderedCoolantTempF = localCoolantTempF;
  }

  if (valuesChanged(localOilPsi, lastRenderedOilPsi, 0.5f)) {
    drawOilPsiDial(localOilPsi, false);
    lastRenderedOilPsi = localOilPsi;
  }

  if (valuesChanged(localBatteryVoltage, lastRenderedBatteryVoltage, 0.05f)) {
    drawBatteryDial(localBatteryVoltage, false);
    lastRenderedBatteryVoltage = localBatteryVoltage;
  }

  if (valuesChanged(localFuelVoltage, lastRenderedFuelVoltage, 0.01f)) {
    drawFuelVoltageReadout(localFuelVoltage);
    lastRenderedFuelVoltage = localFuelVoltage;
  }

  if (valuesChanged(localSpeedMph, lastRenderedSpeedMph, 0.5f)) {
    drawSpeedDial(localSpeedMph, false);
    lastRenderedSpeedMph = localSpeedMph;
  }

  if (valuesChanged(localOdometerMiles, lastRenderedOdometerMiles, 0.1f)) {
    drawOdometerPanel(localOdometerMiles, false);
    lastRenderedOdometerMiles = localOdometerMiles;
  }
}

void sensorTask(void* parameter) {
  (void)parameter;

  for (;;) {
    uint32_t now = millis();

    if (now - lastSpeedDirectSampleMs >= SPEED_DIRECT_SAMPLE_MS) {
      noInterrupts();
      uint32_t directPulses = speedDirectPulseCount;
      speedDirectPulseCount = 0;
      interrupts();

      // A handful of stray edges shouldn't register as motion.
      if (directPulses < SPEED_DIRECT_MIN_PULSES) {
        directPulses = 0;
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
      Serial.print(speedDirectMph, 1);
      Serial.println();
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

        if (speedMph <= 0.0f && speedDirectMph > 0.0f) {
          speedMph = speedDirectMph;
        }

        speedMph = applySpeedCalibration(speedMph);

        if (!ODOMETER_PAUSED && speedMph > 2.0f) {
          odometerMiles += speedMph * elapsedHours;
          saveOdometerIfNeeded(false);
        } else if (!ODOMETER_PAUSED && odometerMiles > odometerSavedMiles) {
          saveOdometerIfNeeded(true);
        }
      }

      if (pulseCount == 0 && speedDirectPulsesPerSecond <= 0.1f) {
        speedFrequencyHz *= 0.7f;
        if (speedFrequencyHz < SPEED_ZERO_HOLD_HZ) {
          wheelRpm = 0.0f;
          speedMph = 0.0f;
          speedFrequencyHz = 0.0f;
          speedCorrectedFrequencyHz = 0.0f;
          speedPulsesPerSample = 0;
          if (!ODOMETER_PAUSED && odometerMiles > odometerSavedMiles) {
            saveOdometerIfNeeded(true);
          }
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
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void displayTask(void* parameter) {
  (void)parameter;

  for (;;) {
    if (displayMutex != NULL && xSemaphoreTake(displayMutex, portMAX_DELAY) == pdTRUE) {
      renderDashboardFrame();
      xSemaphoreGive(displayMutex);
    }
    vTaskDelay(25 / portTICK_PERIOD_MS);
  }
}

void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}
