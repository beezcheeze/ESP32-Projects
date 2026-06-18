#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define MUX_ADDR 0x70  // Multiplexer address with A0 low, A1 low
#define A0_PIN 32       // GPIO pin connected to A0
#define A1_PIN 33       // GPIO pin connected to A1

// Create OLED display objects
Adafruit_SSD1306 display1(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_SSD1306 display2(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_SSD1306 display3(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_SSD1306 display4(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_SSD1306 display5(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Function to select I2C multiplexer channel
void selectChannel(uint8_t channel) {
  if (channel > 7) return;  // PCA9548A has 8 channels (0-7)
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << channel);  // Select the channel
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  
  // Set multiplexer address pins for 0x70
  pinMode(A0_PIN, OUTPUT);
  pinMode(A1_PIN, OUTPUT);
  digitalWrite(A0_PIN, LOW);   // A0 low
  digitalWrite(A1_PIN, LOW);   // A1 low
  delay(100);  // Allow time for address change
  
  Wire.begin();  // Initialize I2C

  // Initialize first OLED on channel 0
  selectChannel(0);
  if (!display1.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // 0x3C is common I2C address for SSD1306
    Serial.println(F("SSD1306 allocation failed for display 1"));
    for (;;);  // Don't proceed, loop forever
  }
  display1.display();  // Show Adafruit splash screen
  delay(2000);
  display1.clearDisplay();

  // Initialize second OLED on channel 1
  selectChannel(1);
  if (!display2.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // Same address, but different channel
    Serial.println(F("SSD1306 allocation failed for display 2"));
    for (;;);  // Don't proceed, loop forever
  }
  display2.display();  // Show Adafruit splash screen
  delay(2000);
  display2.clearDisplay();

  // Initialize third OLED on channel 2
  selectChannel(2);
  if (!display3.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed for display 3"));
    for (;;);
  }
  display3.display();
  delay(2000);
  display3.clearDisplay();

  // Initialize fourth OLED on channel 3
  selectChannel(3);
  if (!display4.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed for display 4"));
    for (;;);
  }
  display4.display();
  delay(2000);
  display4.clearDisplay();

  // Initialize fifth OLED on channel 4
  selectChannel(4);
  if (!display5.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed for display 5"));
    for (;;);
  }
  display5.display();
  delay(2000);
  display5.clearDisplay();

  Serial.println("OLED initialization complete");
}

void loop() {
  // Display on first OLED (channel 0)
  selectChannel(0);
  display1.clearDisplay();
  display1.setTextSize(2);
  display1.setTextColor(SSD1306_WHITE);
  display1.setCursor(0, 0);
  display1.println("OLED 1");
  display1.setCursor(0, 20);
  display1.println("Screen 1");
  display1.setCursor(0, 40);
  display1.println("GOOD");
  display1.display();

  // Display on second OLED (channel 1)
  selectChannel(1);
  display2.clearDisplay();
  display2.setTextSize(2);
  display2.setTextColor(SSD1306_WHITE);
  display2.setCursor(0, 0);
  display2.println("OLED 2");
  display2.setCursor(0, 20);
  display2.println("Screen 2");
  display2.setCursor(0, 40);
  display2.println(millis() / 10);
  display2.display();

  // Display on third OLED (channel 2)
  selectChannel(2);
  display3.clearDisplay();
  display3.setTextSize(2);
  display3.setTextColor(SSD1306_WHITE);
  display3.setCursor(0, 0);
  display3.println("OLED 3");
  display3.setCursor(0, 20);
  display3.println("Screen 3");
  display3.setCursor(0, 40);
  display3.println(millis() / 100);
  display3.display();

  // Display on fourth OLED (channel 3)
  selectChannel(3);
  display4.clearDisplay();
  display4.setTextSize(2);
  display4.setTextColor(SSD1306_WHITE);
  display4.setCursor(0, 0);
  display4.println("OLED 4");
  display4.setCursor(0, 20);
  display4.println("Screen 4");
  display4.setCursor(0, 40);
  display4.println(millis() / 1000);
  display4.display();

  // Display on fifth OLED (channel 4)
  selectChannel(4);
  display5.clearDisplay();
  display5.setTextSize(2);
  display5.setTextColor(SSD1306_WHITE);
  display5.setCursor(0, 0);
  display5.println("OLED 5");
  display5.setCursor(0, 20);
  display5.println("Screen 5");
  display5.setCursor(0, 40);
  display5.println(millis() / 10000);
  display5.display();

  delay(1000);  // Update every second
}