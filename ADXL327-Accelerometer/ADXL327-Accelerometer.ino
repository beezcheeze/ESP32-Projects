#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const int xPin = 36;
const int yPin = 39;
const int zPin = 34;

void setup() {
  Serial.begin(115200); // Higher baud rate recommended for ESP32
  // Set ADC attenuation to 11dB to read the full 0-3.3V range
  analogSetAttenuation(ADC_11db); 
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.clearDisplay();
  display.setTextSize(2); //max of 8 to Y axis
  display.setTextColor(WHITE);
}

void loop() {
  int xRaw = analogRead(xPin);
  int yRaw = analogRead(yPin);
  int zRaw = analogRead(zPin);

  // Convert raw 12-bit value (0-4095) to Voltage (0-3.3V)
  float xmV = xRaw * (3300 / 4095.0);
  float ymV = yRaw * (3300 / 4095.0);
  float zmV = zRaw * (3300 / 4095.0);

  float xG = (xmV - 1500) / 420; // Assuming 1g = 420mV
  float yG = (ymV - 1473) / 420;
  float zG = (zmV - 1640) / 420;

  Serial.printf("X: %d (%.2fmV)\tY: %d (%.2fmV)\tZ: %d (%.2fmV)\n", xRaw, xmV, yRaw, ymV, zRaw, zmV);
  display.clearDisplay();
  display.setCursor(0,0);
  display.printf("X: %.2fg\nY: %.2fg\nZ: %.2fg", xG, yG, zG);
  display.display();
  delay(100);
}
