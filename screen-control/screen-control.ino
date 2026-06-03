#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int count = 0;

void setup() {
  Serial.begin(115200);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }

  display.clearDisplay();
  display.setTextSize(4); //max of 8 to Y axis
  display.setTextColor(WHITE);
}

void loop() {;
  display.clearDisplay();
  display.setCursor(20,0);
  display.printf("RPM: %d", count);
  display.display();
  count = count + 10;
  if(count > 5500) count = 0;
}