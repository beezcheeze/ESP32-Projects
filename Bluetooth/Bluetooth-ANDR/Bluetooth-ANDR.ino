#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define MUX_ADDR 0x70  // Multiplexer address with A0 low, A1 low
#define A0_PIN 32       // GPIO pin connected to A0
#define A1_PIN 33       // GPIO pin connected to A1

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

Adafruit_SSD1306 display1(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_SSD1306 display2(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_SSD1306 display3(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;

String screenText[3] = {"Screen 1", "Boost PSI", "Fuel PSI"};

void selectChannel(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void drawScreen(Adafruit_SSD1306 &display, uint8_t channel, const String &line1, const String &line2, const String &line3) {
  selectChannel(channel);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(line1);
  display.setTextSize(1);
  display.setCursor(0, 24);
  display.println(line2);
  display.setCursor(0, 36);
  display.println(line3);
  display.display();
}

void updateDisplayFromCommand(uint8_t idx, const String &payload) {
  String text = payload;
  if (text.length() == 0) return;
  screenText[idx] = text;
  if (idx == 0) drawScreen(display1, 0, "OLED 1", text, "");
  if (idx == 1) drawScreen(display2, 1, "OLED 2", text, "");
  if (idx == 2) drawScreen(display3, 2, "OLED 3", text, "");
}

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) override {
      deviceConnected = true;
    }

    void onDisconnect(BLEServer* pServer) override {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) override {
      String value = pCharacteristic->getValue();
      Serial.print("BLE RX: ");
      Serial.println(value);
      if (value.length() == 0) return;

      if (value.startsWith("S1:")) {
        updateDisplayFromCommand(0, value.substring(3));
      } else if (value.startsWith("S2:")) {
        updateDisplayFromCommand(1, value.substring(3));
      } else if (value.startsWith("S3:")) {
        updateDisplayFromCommand(2, value.substring(3));
      } else if (value == "1") {
        digitalWrite(2, HIGH);
        Serial.println("GPIO2 HIGH");
      } else if (value == "0") {
        digitalWrite(2, LOW);
        Serial.println("GPIO2 LOW");
      } else {
        Serial.println("Unknown command");
      }
    }
};

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE Android UART with OLED...");

  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);

  pinMode(A0_PIN, OUTPUT);
  pinMode(A1_PIN, OUTPUT);
  digitalWrite(A0_PIN, LOW);
  digitalWrite(A1_PIN, LOW);
  delay(100);

  Wire.begin();

  selectChannel(0);
  if (!display1.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed for display 1"));
    for (;;);
  }
  selectChannel(1);
  if (!display2.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed for display 2"));
    for (;;);
  }
  selectChannel(2);
  if (!display3.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed for display 3"));
    for (;;);
  }

  drawScreen(display1, 0, "OLED 1", screenText[0], "");
  drawScreen(display2, 1, "OLED 2", screenText[1], "");
  drawScreen(display3, 2, "OLED 3", screenText[2], "");

  BLEDevice::init("ESP32-BLE-ANDROID");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pTxCharacteristic = pService->createCharacteristic(
                          CHARACTERISTIC_UUID_TX,
                          BLECharacteristic::PROPERTY_NOTIFY
                        );
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_RX,
                                         BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
                                       );
  pRxCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  pAdvertising->start();

  Serial.begin(115200);
  Serial.println("BLE Android UART with OLED ready");
  Serial.println("Send S1:<text>, S2:<text>, S3:<text>, 1 or 0");
}

void loop() {
  static unsigned long lastNotify = 0;
  if (millis() - lastNotify > 1000) {
    lastNotify = millis();
    if (deviceConnected) {
      String msg = "BLE connected\n";
      pTxCharacteristic->setValue(msg.c_str());
      pTxCharacteristic->notify();
    }
  }
  delay(20);
}
