#include <BluetoothSerial.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <esp_gap_bt_api.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define MUX_ADDR 0x70  // PCA9548A I2C multiplexer address (A2/A1/A0 low)
#define A0_PIN 32       // Multiplexer address pin A0
#define A1_PIN 33       // Multiplexer address pin A1
#define A2_PIN 25       // Multiplexer address pin A2
#define BLUE_LED_PIN 2  // Typical ESP32 onboard blue LED

// Analog pins for a 3-axis accelerometer module (adjust as needed)
const int accelXPin = 36;
const int accelYPin = 39;
const int accelZPin = 34;

// Change this to the Bluetooth name of your BAFX OBD-II adapter if needed.
const char *obdDeviceName = "OBDII";

BluetoothSerial obdSerial;
Adafruit_SSD1306 display1(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_SSD1306 display2(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_SSD1306 display3(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_SSD1306 display4(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_SSD1306 display5(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
volatile bool connectModeActive = false;
volatile bool ledState = false;
volatile bool displaysReady = false;
volatile unsigned long connectModeStartMs = 0;
volatile uint8_t connectStage = 0; // 0 = BT Init, 1 = Connecting
volatile bool obdConnected = false;
volatile bool btScanActive = false;

static const uint8_t MAX_BT_DEVICES = 4;
static const uint32_t BT_SCAN_INTERVAL_MS = 30000UL;
static const uint8_t BT_SCAN_DURATION_SECONDS = 8;

struct BtDeviceEntry {
  char label[32];
  int8_t rssi;
  bool hasRssi;
  bool valid;
};

static BtDeviceEntry btDevices[MAX_BT_DEVICES];
static unsigned long lastBtScanStartMs = 0;

const unsigned long LED_BLINK_INTERVAL_MS = 250;
const unsigned long LOADING_DRAW_INTERVAL_MS = 1;
const uint8_t OLED_INIT_RETRIES = 3;
const unsigned long OLED_BOOT_SPLASH_MS = 900;

static const uint8_t SCREEN1_CHANNEL = 0;
static const uint8_t SCREEN2_CHANNEL = 1;
static const uint8_t SCREEN3_CHANNEL = 2;
static const uint8_t SCREEN4_CHANNEL = 3;
static const uint8_t SCREEN5_CHANNEL = 4;

void showConnectCounterScreen(Adafruit_SSD1306 &disp, const char *stageTitle, unsigned long elapsedMs);
void showLoading(Adafruit_SSD1306 &disp, const char *title, unsigned long elapsedMs);
void showSubaruBootScreen(Adafruit_SSD1306 &disp, unsigned long elapsedMs);
void showVoltageScreen(Adafruit_SSD1306 &disp, float voltage, unsigned long uptime);
void showRuntimeScreen(Adafruit_SSD1306 &disp, unsigned long uptime, bool connected);
void showMessage(Adafruit_SSD1306 &disp, const char *line1, const char *line2);
bool initOledOnChannel(uint8_t channel, Adafruit_SSD1306 &disp, const char *label);
void bluetoothScanTask(void *parameter);
void btGapCallback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
void clearBtDevices();
void storeBtDevice(const char *label, int8_t rssi, bool hasRssi);
void formatBtAddress(const uint8_t *bda, char *buffer, size_t bufferSize);
void maybeStartBluetoothScan();
bool isOledPresentOnChannel(uint8_t channel);
void detectExtraScreenChannels();

void selectChannel(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void disableAllChannels() {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
}

bool initOledOnChannel(uint8_t channel, Adafruit_SSD1306 &disp, const char *label) {
  Serial.print("Initializing ");
  Serial.println(label);

  for (uint8_t attempt = 1; attempt <= OLED_INIT_RETRIES; attempt++) {
    disableAllChannels();
    delay(15);
    selectChannel(channel);
    delay(25);

    if (disp.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
      disp.clearDisplay();
      disp.setTextColor(SSD1306_WHITE);
      disp.setTextSize(1);
      disp.setCursor(0, 0);
      disp.println("ESP32 OBD-II");
      disp.setCursor(0, 14);
      disp.print("Screen ");
      disp.println(channel + 1);
      disp.setCursor(0, 28);
      disp.println("Booting...");
      disp.display();
      delay(OLED_BOOT_SPLASH_MS);
      return true;
    }

    Serial.print("Init failed on ");
    Serial.print(label);
    Serial.print(" (attempt ");
    Serial.print(attempt);
    Serial.println(")");
    delay(80);
  }

  return false;
}

bool isOledPresentOnChannel(uint8_t channel) {
  disableAllChannels();
  delay(2);
  selectChannel(channel);
  delay(2);
  Wire.beginTransmission(0x3C);
  return Wire.endTransmission() == 0;
}

void detectExtraScreenChannels() {
  Serial.print("Screen channel map: ");
  Serial.print(SCREEN1_CHANNEL);
  Serial.print(", ");
  Serial.print(SCREEN2_CHANNEL);
  Serial.print(", ");
  Serial.print(SCREEN3_CHANNEL);
  Serial.print(", ");
  Serial.print(SCREEN4_CHANNEL);
  Serial.print(", ");
  Serial.println(SCREEN5_CHANNEL);
}

void clearBtDevices() {
  for (uint8_t i = 0; i < MAX_BT_DEVICES; i++) {
    btDevices[i].label[0] = '\0';
    btDevices[i].rssi = 0;
    btDevices[i].hasRssi = false;
    btDevices[i].valid = false;
  }
}

void storeBtDevice(const char *label, int8_t rssi, bool hasRssi) {
  for (uint8_t i = 0; i < MAX_BT_DEVICES; i++) {
    if (btDevices[i].valid && strcmp(btDevices[i].label, label) == 0) {
      btDevices[i].rssi = rssi;
      btDevices[i].hasRssi = hasRssi;
      return;
    }
  }

  for (uint8_t i = 0; i < MAX_BT_DEVICES; i++) {
    if (!btDevices[i].valid) {
      strncpy(btDevices[i].label, label, sizeof(btDevices[i].label) - 1);
      btDevices[i].label[sizeof(btDevices[i].label) - 1] = '\0';
      btDevices[i].rssi = rssi;
      btDevices[i].hasRssi = hasRssi;
      btDevices[i].valid = true;
      return;
    }
  }
}

void formatBtAddress(const uint8_t *bda, char *buffer, size_t bufferSize) {
  snprintf(
    buffer,
    bufferSize,
    "%02X:%02X:%02X:%02X:%02X:%02X",
    bda[0],
    bda[1],
    bda[2],
    bda[3],
    bda[4],
    bda[5]
  );
}

void btGapCallback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
  switch (event) {
    case ESP_BT_GAP_DISC_RES_EVT: {
      char label[32];
      label[0] = '\0';
      int8_t rssi = 0;
      bool hasRssi = false;
      bool hasLabel = false;

      for (uint32_t i = 0; i < param->disc_res.num_prop; i++) {
        const esp_bt_gap_dev_prop_t &prop = param->disc_res.prop[i];
        if (prop.type == ESP_BT_GAP_DEV_PROP_RSSI && prop.len >= 1) {
          rssi = *((int8_t *)prop.val);
          hasRssi = true;
        }

        if (prop.type == ESP_BT_GAP_DEV_PROP_EIR && prop.val != nullptr && prop.len > 0) {
          uint8_t nameLen = 0;
          uint8_t *namePtr = esp_bt_gap_resolve_eir_data((uint8_t *)prop.val, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &nameLen);
          if (namePtr == nullptr) {
            namePtr = esp_bt_gap_resolve_eir_data((uint8_t *)prop.val, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &nameLen);
          }

          if (namePtr != nullptr && nameLen > 0) {
            size_t copyLen = nameLen;
            if (copyLen >= sizeof(label)) {
              copyLen = sizeof(label) - 1;
            }
            memcpy(label, namePtr, copyLen);
            label[copyLen] = '\0';
            hasLabel = true;
          }
        }
      }

      if (!hasLabel) {
        formatBtAddress(param->disc_res.bda, label, sizeof(label));
      }

      storeBtDevice(label, rssi, hasRssi);
      break;
    }

    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
      btScanActive = (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED);
      if (!btScanActive) {
        lastBtScanStartMs = millis();
      }
      break;

    default:
      break;
  }
}

void maybeStartBluetoothScan() {
  if (btScanActive) {
    return;
  }

  unsigned long now = millis();
  unsigned long minIntervalMs = obdConnected ? BT_SCAN_INTERVAL_MS : 5000UL;
  if (lastBtScanStartMs != 0 && (now - lastBtScanStartMs) < minIntervalMs) {
    return;
  }

  clearBtDevices();
  lastBtScanStartMs = now;
  esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, BT_SCAN_DURATION_SECONDS, 0);
}

void bluetoothScanTask(void *parameter) {
  (void)parameter;

  while (true) {
    maybeStartBluetoothScan();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void setBlueLed(bool on) {
  digitalWrite(BLUE_LED_PIN, on ? HIGH : LOW);
}

void ledBlinkTask(void *parameter) {
  (void)parameter;
  while (true) {
    if (connectModeActive) {
      ledState = !ledState;
      setBlueLed(ledState);
      vTaskDelay(pdMS_TO_TICKS(LED_BLINK_INTERVAL_MS));
    } else {
      vTaskDelay(pdMS_TO_TICKS(20));
    }
  }
}

void loadingBarTask(void *parameter) {
  (void)parameter;

  while (true) {
    if (connectModeActive && displaysReady) {
      unsigned long now = millis();
      unsigned long elapsed = now - connectModeStartMs;

      const char *title = (connectStage == 0) ? "BT Init" : "Connecting";

      selectChannel(SCREEN1_CHANNEL);
      showConnectCounterScreen(display1, title, elapsed);

      selectChannel(SCREEN2_CHANNEL);
      showLoading(display2, title, elapsed);

      selectChannel(SCREEN3_CHANNEL);
      showSubaruBootScreen(display3, elapsed);

      // Keep remaining screens alive during connect mode so they are not blank.
      selectChannel(SCREEN4_CHANNEL);
      showMessage(display4, "OBD Link", "Waiting...");

      selectChannel(SCREEN5_CHANNEL);
      showRuntimeScreen(display5, elapsed / 1000UL, obdConnected);

      vTaskDelay(pdMS_TO_TICKS(LOADING_DRAW_INTERVAL_MS));
    } else {
      vTaskDelay(pdMS_TO_TICKS(20));
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(A0_PIN, OUTPUT);
  pinMode(A1_PIN, OUTPUT);
  pinMode(A2_PIN, OUTPUT);
  digitalWrite(A0_PIN, LOW);
  digitalWrite(A1_PIN, LOW);
  digitalWrite(A2_PIN, LOW);
  pinMode(BLUE_LED_PIN, OUTPUT);
  setBlueLed(false);
  xTaskCreatePinnedToCore(ledBlinkTask, "ledBlinkTask", 2048, nullptr, 1, nullptr, 1);
  delay(100);

  Wire.begin();
  Wire.setClock(100000);
  disableAllChannels();
  detectExtraScreenChannels();
  analogReadResolution(12);

  if (!initOledOnChannel(SCREEN1_CHANNEL, display1, "display 1")) {
    Serial.println("SSD1306 allocation failed for display 1");
    while (true) {
      delay(1000);
    }
  }

  if (!initOledOnChannel(SCREEN2_CHANNEL, display2, "display 2")) {
    Serial.println("SSD1306 allocation failed for display 2");
    while (true) {
      delay(1000);
    }
  }

  if (!initOledOnChannel(SCREEN3_CHANNEL, display3, "display 3")) {
    Serial.println("SSD1306 allocation failed for display 3");
    while (true) {
      delay(1000);
    }
  }

  if (!initOledOnChannel(SCREEN4_CHANNEL, display4, "display 4")) {
    Serial.println("SSD1306 allocation failed for display 4");
    while (true) {
      delay(1000);
    }
  }

  if (!initOledOnChannel(SCREEN5_CHANNEL, display5, "display 5")) {
    Serial.println("SSD1306 allocation failed for display 5");
    while (true) {
      delay(1000);
    }
  }

  selectChannel(SCREEN1_CHANNEL);
  display1.clearDisplay();
  display1.setTextColor(SSD1306_WHITE);
  display1.setTextSize(1);
  display1.setCursor(0, 0);
  display1.println("ESP32 OBD-II\nMAP Reader");
  display1.println("Connecting To OBD II");
  display1.display();

  selectChannel(SCREEN2_CHANNEL);
  display2.clearDisplay();
  display2.display();

  selectChannel(SCREEN3_CHANNEL);
  display3.clearDisplay();
  display3.display();

  selectChannel(SCREEN4_CHANNEL);
  display4.clearDisplay();
  display4.display();

  selectChannel(SCREEN5_CHANNEL);
  display5.clearDisplay();
  display5.display();

  disableAllChannels();
  delay(100);

  displaysReady = true;
  xTaskCreatePinnedToCore(loadingBarTask, "loadingBarTask", 3072, nullptr, 1, nullptr, 1);

  Serial.println("Initializing Bluetooth...");
  connectModeActive = true;
  connectModeStartMs = millis();
  bool btReady = false;
  connectStage = 0;
  while (!btReady) {
    btReady = obdSerial.begin("ESP32_OBD", true);
    if (!btReady) {
      Serial.println("Bluetooth init failed, retrying...");
      delay(300);
    }
  }

  // Start discovery support immediately so nearby devices can be listed
  // while OBD connection attempts are still in progress.
  esp_bt_gap_register_callback(btGapCallback);
  clearBtDevices();
  lastBtScanStartMs = 0;
  xTaskCreatePinnedToCore(bluetoothScanTask, "bluetoothScanTask", 4096, nullptr, 1, nullptr, 1);

  bool connected = false;
  connectStage = 1;
  while (!connected) {
    connected = obdSerial.connect(obdDeviceName);
    if (!connected) {
      delay(500);
    }
  }

  connectModeActive = false;
  setBlueLed(true);
  obdConnected = true;

  delay(100);
  selectChannel(SCREEN2_CHANNEL);
  display2.clearDisplay();
  display2.display();

  Serial.print("Connected to ");
  Serial.println(obdDeviceName);

  runInitCommand("ATZ");      // Reset adapter
  runInitCommand("ATE0");     // Echo off
  runInitCommand("ATL0");     // Linefeeds off
  runInitCommand("ATS0");     // Spaces off
  runInitCommand("ATH0");     // Headers off
  runInitCommand("ATSP0");    // Automatic protocol
}

void loop() {
  float mapKpa = readManifoldAbsolutePressure();
  float mapPsi = mapKpa >= 0 ? mapKpa * 0.145038f : 0.0f;
  float mapInHg = mapKpa >= 0 ? mapKpa * 0.295300f : 0.0f;
  float accelX = readAccelAxis(accelXPin);
  float accelY = readAccelAxis(accelYPin);
  float accelZ = readAccelAxis(accelZPin);
  unsigned long uptime = millis() / 1000;

  selectChannel(SCREEN1_CHANNEL);
  delay(50);
  Serial.println("Updating screen 0");
  showMapScreen(display1, mapKpa, mapInHg, mapPsi);

  selectChannel(SCREEN2_CHANNEL);
  delay(50);
  Serial.println("Updating screen 1");
  showStatusScreen(display2, uptime, mapKpa >= 0);

  selectChannel(SCREEN3_CHANNEL);
  delay(50);
  Serial.println("Updating screen 2");
  showAccelScreen(display3, accelX, accelY, accelZ);

  selectChannel(SCREEN4_CHANNEL);
  delay(50);
  Serial.println("Updating screen 3");
  showVoltageScreen(display4, mapKpa >= 0 ? (mapKpa / 100.0f) : 0.0f, uptime);

  selectChannel(SCREEN5_CHANNEL);
  delay(50);
  Serial.println("Updating screen 4");
  showRuntimeScreen(display5, uptime, obdConnected);

  delay(1000);
}

void showMapScreen(Adafruit_SSD1306 &disp, float mapKpa, float mapInHg, float mapPsi) {
  disp.clearDisplay();
  disp.setTextColor(SSD1306_WHITE);
  disp.setTextSize(2);
  disp.setCursor(0, 0);
  disp.println("MAP");

  disp.setTextSize(3);
  disp.setCursor(0, 24);
  if (mapKpa >= 0) {
    disp.print(String(mapKpa, 1));
  } else {
    disp.print("ERR");
  }

  disp.setTextSize(1);
  disp.setCursor(0, 52);
  if (mapKpa >= 0) {
    disp.print(String(mapInHg, 1));
    disp.print(" inHg ");
    disp.print(String(mapPsi, 2));
    disp.print(" psi");
  } else {
    disp.print("MAP read failed");
  }
  disp.display();
}

void showAccelScreen(Adafruit_SSD1306 &disp, float accelX, float accelY, float accelZ) {
  disp.clearDisplay();
  disp.setTextColor(SSD1306_WHITE);
  disp.setTextSize(2);
  disp.setCursor(0, 0);
  disp.println("Accel");

  disp.setTextSize(1);
  disp.setCursor(0, 20);
  disp.print("X: ");
  disp.print(String(accelX, 2));
  disp.print("g");

  disp.setCursor(0, 34);
  disp.print("Y: ");
  disp.print(String(accelY, 2));
  disp.print("g");

  disp.setCursor(0, 48);
  disp.print("Z: ");
  disp.print(String(accelZ, 2));
  disp.print("g");
  disp.display();
}

void showStatusScreen(Adafruit_SSD1306 &disp, unsigned long uptime, bool obdOk) {
  disp.clearDisplay();
  disp.setTextColor(SSD1306_WHITE);
  disp.setTextSize(1);
  disp.setCursor(0, 0);
  disp.println("BT Status");

  disp.setCursor(0, 14);
  disp.println("Link: Connected");

  disp.setCursor(0, 30);
  disp.print("OBD:");
  disp.print(obdOk ? "OK " : "ERR ");
  disp.print("Up:");
  disp.print(uptime);
  disp.println("s");

  disp.display();
}

void showVoltageScreen(Adafruit_SSD1306 &disp, float voltage, unsigned long uptime) {
  disp.clearDisplay();
  disp.setTextColor(SSD1306_WHITE);
  disp.setTextSize(2);
  disp.setCursor(0, 0);
  disp.println("VOLT");

  disp.setTextSize(3);
  disp.setCursor(0, 24);
  disp.print(String(voltage, 2));

  disp.setTextSize(1);
  disp.setCursor(0, 52);
  disp.print("Up:");
  disp.print(uptime);
  disp.print("s");
  disp.display();
}

void showRuntimeScreen(Adafruit_SSD1306 &disp, unsigned long uptime, bool connected) {
  disp.clearDisplay();
  disp.setTextColor(SSD1306_WHITE);
  disp.setTextSize(1);
  disp.setCursor(0, 0);
  disp.println("BT Devices");

  disp.setTextSize(1);
  disp.setCursor(0, 12);
  disp.print("OBD: ");
  disp.println(connected ? "CONNECTED" : "WAITING");

  if (btScanActive) {
    disp.setCursor(80, 12);
    disp.print("SCAN");
  }

  bool drewAny = false;
  uint8_t row = 0;
  for (uint8_t i = 0; i < MAX_BT_DEVICES; i++) {
    if (!btDevices[i].valid) {
      continue;
    }

    uint8_t y = 24 + (row * 10);
    if (y > 48) {
      break;
    }

    disp.setCursor(0, y);
    disp.print(i + 1);
    disp.print(": ");
    disp.print(btDevices[i].label);
    if (btDevices[i].hasRssi) {
      disp.print(" ");
      disp.print((int)btDevices[i].rssi);
      disp.print("dBm");
    }
    drewAny = true;
    row++;
  }

  if (!drewAny) {
    disp.setCursor(0, 28);
    disp.println(btScanActive ? "Scanning..." : "No devices found");
  }

  disp.setCursor(0, 56);
  disp.print("Up:");
  disp.print(uptime);
  disp.print("s");
  disp.display();
}

float readManifoldAbsolutePressure() {
  String response = queryOBD("01 0B");
  if (response.length() == 0) {
    return -1.0;
  }

  int rawValue = parseOBDResponse(response, 0x0B);
  if (rawValue < 0) {
    return -1.0;
  }

  return float(rawValue);
}

String sanitizeResponse(const String &resp) {
  String s = resp;
  s.replace("\r", "");
  s.replace("\n", "");
  s.replace(">", "");
  s.trim();
  return s;
}

int parseOBDResponse(const String &resp, uint8_t pid) {
  String s = sanitizeResponse(resp);
  s.replace(" ", "");
  s.toUpperCase();

  String pidHex = String(pid, HEX);
  if (pidHex.length() == 1) {
    pidHex = "0" + pidHex;
  }
  String expected = "41" + pidHex;

  int idx = s.indexOf(expected);
  if (idx < 0 || idx + expected.length() + 2 > s.length()) {
    return -1;
  }

  String hexByte = s.substring(idx + expected.length(), idx + expected.length() + 2);
  char buffer[3] = { hexByte[0], hexByte[1], '\0' };
  return (int)strtol(buffer, nullptr, 16);
}

String queryOBD(const char *command) {
  sendOBDCommand(command);
  return readOBDResponse(2000);
}

void sendOBDCommand(const char *command) {
  obdSerial.print(command);
  obdSerial.print("\r");
}

String readOBDResponse(unsigned long timeoutMs) {
  String response;
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    while (obdSerial.available()) {
      char c = obdSerial.read();
      response += c;
      if (c == '>') {
        return sanitizeResponse(response);
      }
    }
  }
  return sanitizeResponse(response);
}

void runInitCommand(const char *command) {
  String resp = queryOBD(command);
  Serial.print(command);
  Serial.print(" -> ");
  Serial.println(resp);
}

void showMessage(Adafruit_SSD1306 &disp, const char *line1, const char *line2) {
  disp.clearDisplay();
  disp.setTextSize(1);
  disp.setCursor(0, 16);
  disp.println(line1);
  disp.println();
  disp.println(line2);
  disp.display();
}

void showSubaruBootScreen(Adafruit_SSD1306 &disp, unsigned long elapsedMs) {
  disp.clearDisplay();
  disp.setTextColor(SSD1306_WHITE);
  (void)elapsedMs;
  disp.setTextSize(1);
  disp.setTextColor(SSD1306_WHITE);
  disp.setTextSize(1);
  disp.setCursor(0, 0);
  disp.print("Vehicle\nIdentification:\n");
  disp.print("2002 SUBARU WRX");
  disp.display();
}

void drawBluetoothModuleIcon(Adafruit_SSD1306 &disp, int x, int y) {
  // Module body.
  disp.drawRoundRect(x, y, 30, 28, 3, SSD1306_WHITE);
  disp.drawRect(x + 2, y + 2, 26, 20, SSD1306_WHITE);

  // Header pins.
  for (int i = 0; i < 6; i++) {
    int px = x + 4 + (i * 4);
    disp.drawLine(px, y + 24, px, y + 27, SSD1306_WHITE);
  }

  // Bluetooth rune.
  int cx = x + 15;
  int top = y + 5;
  int mid = y + 12;
  int bot = y + 19;
  disp.drawLine(cx, top, cx, bot, SSD1306_WHITE);
  disp.drawLine(cx, top, cx + 5, mid - 2, SSD1306_WHITE);
  disp.drawLine(cx, bot, cx + 5, mid + 2, SSD1306_WHITE);
  disp.drawLine(cx + 5, mid - 2, cx, mid, SSD1306_WHITE);
  disp.drawLine(cx + 5, mid + 2, cx, mid, SSD1306_WHITE);

  // Tiny status LED marks.
  disp.fillCircle(x + 4, y + 4, 1, SSD1306_WHITE);
  disp.fillCircle(x + 8, y + 4, 1, SSD1306_WHITE);
}

void showConnectCounterScreen(Adafruit_SSD1306 &disp,
                              const char *stageTitle,
                              unsigned long elapsedMs) {
  unsigned long elapsedSec = elapsedMs / 1000UL;
  unsigned long minutes = elapsedSec / 60UL;
  unsigned long seconds = elapsedSec % 60UL;

  disp.clearDisplay();
  disp.setTextColor(SSD1306_WHITE);
  disp.setTextSize(1);
  disp.setCursor(0, 0);
  disp.println("OBD II");
  disp.setCursor(0, 10);
  disp.print("Status: ");
  disp.println(stageTitle);

  disp.setCursor(0, 22);
  disp.print("Vehicle Status: OFF");

  disp.setCursor(0, 34);
  disp.print("");

  disp.setCursor(0, 56);
  disp.print("Time ");
  if (minutes < 10) disp.print('0');
  disp.print(minutes);
  disp.print(':');
  if (seconds < 10) disp.print('0');
  disp.print(seconds);

  // Use the right-side free area for a small Bluetooth module illustration.
  drawBluetoothModuleIcon(disp, 94, 36);
  disp.display();
}

void showLoading(Adafruit_SSD1306 &disp, const char *title, unsigned long elapsedMs) {
  disp.clearDisplay();

  // Reserve bottom area for a Bluetooth status label.
  const int graphX = 0;
  const int graphY = 0;
  const int graphW = 120;
  const int graphH = 50;
  const int graphInnerX = graphX + 1;
  const int graphInnerY = graphY + 1;
  const int graphInnerW = graphW - 2;
  const int graphInnerH = graphH - 2;

  // Draw only basic x and y axes (no top/right frame lines).
  disp.drawFastHLine(graphInnerX, graphInnerY + graphInnerH - 1, graphInnerW, SSD1306_WHITE);
  disp.drawFastVLine(graphInnerX, graphInnerY, graphInnerH, SSD1306_WHITE);

  // Pseudo-3D axis offset for depth effect.
  const int depthX = 2;
  const int depthY = 2;
  int axisBackX0 = graphInnerX + depthX;
  int axisBackY0 = graphInnerY + depthY;
  int axisBackX1 = graphInnerX + graphInnerW - 1 + depthX;
  int axisBackY1 = graphInnerY + graphInnerH - 1 + depthY;
  if (axisBackX1 < SCREEN_WIDTH && axisBackY1 < SCREEN_HEIGHT) {
    disp.drawFastHLine(axisBackX0, axisBackY1, graphInnerW, SSD1306_WHITE);
    disp.drawFastVLine(axisBackX0, axisBackY0, graphInnerH, SSD1306_WHITE);
    disp.drawLine(graphInnerX, graphInnerY, axisBackX0, axisBackY0, SSD1306_WHITE);
    disp.drawLine(graphInnerX, graphInnerY + graphInnerH - 1, axisBackX0, axisBackY1, SSD1306_WHITE);
    disp.drawLine(graphInnerX + graphInnerW - 1, graphInnerY + graphInnerH - 1, axisBackX1, axisBackY1, SSD1306_WHITE);
  }

  int startX = graphInnerX;
  int startY = graphInnerY + graphInnerH - 1;
  int maxDx = graphInnerW - 1;
  int currentDx = maxDx;
  int maxX = graphInnerX + graphInnerW - 1;
  int endX = startX;
  int endY = startY;

  int prevX = startX;
  int prevY = startY;
  const float harmonicNorm = 1.0f + (1.0f / 3.0f) + (1.0f / 5.0f) + (1.0f / 7.0f) + (1.0f / 9.0f);
  const float baseCyclesAcrossGraph = 2.0f;
  const float elapsedSecF = (float)elapsedMs / 1000.0f;
  const float phaseRateRadPerSec = 1.25f;
  const float attemptPhaseShift = elapsedSecF * phaseRateRadPerSec;
  const float baseDivisor = 3.5f;

  for (int dx = 1; dx <= currentDx; dx++) {
    float phase = (2.0f * PI * baseCyclesAcrossGraph * (float)dx / (float)(maxDx + 1)) + attemptPhaseShift;
    float series = sinf(phase) // / baseDivisor
        + (1.0f / 3.0f) * sinf(3.0f * phase)
        + (1.0f / 5.0f) * sinf(5.0f * phase)
        + (1.0f / 7.0f) * sinf(7.0f * phase)
        + (1.0f / 9.0f) * sinf(9.0f * phase);

    // Normalize to 0..1 for display coordinates.
    float normalized = 0.5f + 0.45f * (series / harmonicNorm);
    if (normalized < 0.0f) normalized = 0.0f;
    if (normalized > 1.0f) normalized = 1.0f;

    int yOffset = (int)(normalized * (float)(graphInnerH - 1));
    int x = startX + dx;
    if (x > maxX) x = maxX;
    int y = startY - yOffset;

    // Draw a shifted secondary trace for a pseudo-3D look.
    int sx = x + depthX;
    int sy = y + depthY;
    if (sx >= 0 && sx < SCREEN_WIDTH && sy >= 0 && sy < SCREEN_HEIGHT) {
      if ((dx & 1) == 0) {
        disp.drawPixel(sx, sy, SSD1306_WHITE);
      }
      if ((dx % 8) == 0) {
        disp.drawLine(x, y, sx, sy, SSD1306_WHITE);
      }
    }

    disp.drawLine(prevX, prevY, x, y, SSD1306_WHITE);
    prevX = x;
    prevY = y;
    endX = x;
    endY = y;
  }
  int markerX = endX - 1;
  if (markerX < graphInnerX) markerX = graphInnerX;
  if (markerX > maxX - 2) markerX = maxX - 2;
  disp.fillRect(markerX, endY - 1, 3, 3, SSD1306_WHITE);

  disp.setTextSize(1);
  disp.setCursor(0, 54);
  disp.print("BT: ");
  disp.print(title);

  disp.display();
}

float readAccelAxis(int pin) {
  int raw = analogRead(pin);
  float voltage = raw * 3.3f / 4095.0f;
  const float accelZeroVoltage = 1.65f;
  const float accelScale = 0.330f; // adjust to your accelerometer's 0g sensitivity
  return (voltage - accelZeroVoltage) / accelScale;
}

