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
const uint32_t SPEED_SAMPLE_WINDOW_MS = 500;
const uint32_t SPEED_MIN_PULSE_GAP_US = 120;
const int SPEED_INTERRUPT_MODE = RISING;
const bool SPEED_USE_INTERNAL_PULLUP = false;
const float SPEED_FREQ_FILTER_ALPHA = 0.25f;
const float SPEED_ZERO_HOLD_HZ = 0.5f;
const float SPEED_IDLE_BIAS_HZ = 350.0f;

#define BG_COLOR      0x0000  // black
#define LABEL_COLOR   0xFFFF  // white
#define VALUE_COLOR   0xFFE0  // yellow
#define BAR_COLOR     0x001F  // blue

int fuelPercent = 0;
volatile uint32_t speedPulseCount = 0;
volatile uint32_t speedLastPulseUs = 0;
float speedMph = 0.0f;
float wheelRpm = 0.0f;
uint32_t lastSpeedSampleMs = 0;
uint32_t speedPulsesPerSample = 0;
float speedFrequencyHz = 0.0f;
float speedRawFrequencyHz = 0.0f;
float speedCorrectedFrequencyHz = 0.0f;

// Use the standard SPI hardware pins on ESP32:
// SCK  = 18, MOSI = 23, MISO = 19
Adafruit_ST7796S tft(LCD_CS, LCD_RS, LCD_RST);

void IRAM_ATTR countSpeedPulse() {
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

  drawDashboard();

  lastSpeedSampleMs = millis();
  speedLastPulseUs = micros();

  Serial.println("ST7796 display initialized.");
}

void drawPanel(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
               const char* title, const char* value,
               uint16_t borderColor, uint16_t fillColor) {
  tft.drawRoundRect(x, y, w, h, 8, borderColor);
  tft.fillRoundRect(x + 2, y + 2, w - 4, h - 4, 6, fillColor);

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
  const float R_SENDER_MIN = 10.0f;
  const float R_SENDER_MAX = 70.0f;
  const float R_DIVIDER = 197.0f;
  const float V_SUPPLY = 12.0f;
  const float V_REF = 3.3f;

  int raw = analogRead(FUEL_ADC_PIN);
  float volts = raw * (V_REF / 4095.0f);

  // Convert the divider output back to the sender resistance.
  // Vout = Vin * Rbottom / (Rtop + Rbottom)
  // With a 197 ohm divider from the 12 V sender to 3.3 V ADC input,
  // the sender resistance can be estimated from the measured voltage.
  float senderOhms = (R_DIVIDER * volts) / (V_SUPPLY - volts);

  // Map the sender range to fuel level.
  // 70 ohm = full, 10 ohm = empty (adjust if your sender is opposite).
  float percent = 100.0f * (senderOhms - R_SENDER_MIN) / (R_SENDER_MAX - R_SENDER_MIN);
  percent = 100.0f - percent;
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
  const float tankGallons = 10.0f * (percent / 100.0f);

  char fuelText[24];
  char gallonsText[24];
  snprintf(fuelText, sizeof(fuelText), "%d %%", percent);
  snprintf(gallonsText, sizeof(gallonsText), "%.1f gal", tankGallons);

  tft.fillRect(gap, bottomY, panelW, panelH, BG_COLOR);
  drawPanel(gap, bottomY, panelW, panelH, "FUEL", fuelText, 0x07E0, BG_COLOR);

  tft.setTextColor(ST77XX_WHITE, BG_COLOR);
  tft.setTextSize(1);
  tft.setCursor(gap + 8, bottomY + 58);
  tft.println(gallonsText);
}

void drawSpeedPanel(float mph, float rpm) {
  uint16_t w = tft.width();
  const uint16_t gap = 6;
  const uint16_t panelW = (w - 3 * gap) / 2;
  const uint16_t panelH = 92;
  const uint16_t topY = 6;

  char speedText[24];
  char speedModeText[24];
  char speedPulseText[24];
  char speedRpmText[24];
  snprintf(speedText, sizeof(speedText), "%.0f MPH", mph);
  snprintf(speedModeText, sizeof(speedModeText), "RAW %.0f COR %.0f", speedRawFrequencyHz, speedCorrectedFrequencyHz);
  snprintf(speedPulseText, sizeof(speedPulseText), "PULSES %lu", (unsigned long)speedPulsesPerSample);
  snprintf(speedRpmText, sizeof(speedRpmText), "RPM %.0f", rpm);

  tft.fillRect(gap, topY, panelW, panelH, BG_COLOR);
  drawPanel(gap, topY, panelW, panelH, "SPEED", speedText, 0x07FF, BG_COLOR);

  tft.setTextColor(ST77XX_WHITE, BG_COLOR);
  tft.setTextSize(1);
  tft.setCursor(gap + 8, topY + 58);
  tft.println(speedModeText);

  tft.setCursor(gap + 8, topY + 70);
  tft.println(speedPulseText);

  tft.setCursor(gap + 140, topY + 70);
  tft.println(speedRpmText);
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

  drawSpeedPanel(speedMph, wheelRpm);
  drawPanel(w - gap - panelW, topY, panelW, panelH, "RPM", "3200", 0xF800, BG_COLOR);
  drawFuelPanel(fuelPercent);
  drawPanel(w - gap - panelW, bottomY, panelW, panelH, "TEMP", "190 *F", 0xF81F, BG_COLOR);

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
}

void loop() {
  static uint32_t lastFuelUpdate = 0;

  uint32_t now = millis();

  if (now - lastFuelUpdate >= 500) {
    lastFuelUpdate = now;
    int newFuelPercent = readFuelPercent();

    if (newFuelPercent != fuelPercent) {
      fuelPercent = newFuelPercent;
      drawFuelPanel(fuelPercent);
    }
  }

  if (now - lastSpeedSampleMs >= SPEED_SAMPLE_WINDOW_MS) {
    noInterrupts();
    uint32_t pulseCount = speedPulseCount;
    speedPulseCount = 0;
    interrupts();

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

      speedPulsesPerSample = (uint32_t)((speedCorrectedFrequencyHz * elapsedMs / 1000.0f) + 0.5f);

      float filteredRevolutions = (speedFrequencyHz * elapsedMs / 1000.0f) / SPEED_PULSES_PER_REVOLUTION;
      wheelRpm = filteredRevolutions / elapsedHours / 60.0f;

      if (SPEED_PULSES_PER_MILE > 0.0f) {
        speedMph = (speedFrequencyHz * 3600.0f) / SPEED_PULSES_PER_MILE;
      } else if (WHEEL_CIRCUMFERENCE_INCHES > 0.0f) {
        speedMph = (filteredRevolutions / elapsedHours) * (WHEEL_CIRCUMFERENCE_INCHES / 63360.0f);
      } else {
        speedMph = 0.0f;
      }
    }

    if (pulseCount == 0) {
      speedFrequencyHz *= 0.7f;
      if (speedFrequencyHz < SPEED_ZERO_HOLD_HZ) {
        wheelRpm = 0.0f;
        speedMph = 0.0f;
        speedFrequencyHz = 0.0f;
        speedCorrectedFrequencyHz = 0.0f;
        speedPulsesPerSample = 0;
      }
    }

    Serial.print("Speed pulses=");
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

    drawSpeedPanel(speedMph, wheelRpm);
  }
}
