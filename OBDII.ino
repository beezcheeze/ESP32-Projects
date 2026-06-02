#include <BluetoothSerial.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define MUX_ADDR 0x70  // PCA9548A I2C multiplexer address
#define A0_PIN 32       // Multiplexer address pin A0
#define A1_PIN 33       // Multiplexer address pin A1
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
volatile bool connectModeActive = false;
volatile bool ledState = false;
volatile bool displaysReady = false;
volatile unsigned long connectModeStartMs = 0;
volatile uint8_t connectStage = 0; // 0 = BT Init, 1 = Connecting

const unsigned long LED_BLINK_INTERVAL_MS = 250;
const unsigned long LOADING_DRAW_INTERVAL_MS = 1;
const uint8_t OLED_INIT_RETRIES = 3;
const unsigned long OLED_BOOT_SPLASH_MS = 900;

void showConnectCounterScreen(Adafruit_SSD1306 &disp, const char *stageTitle, unsigned long elapsedMs);
void showLoading(Adafruit_SSD1306 &disp, const char *title, unsigned long elapsedMs);
void showSubaruBootScreen(Adafruit_SSD1306 &disp, unsigned long elapsedMs);
bool initOledOnChannel(uint8_t channel, Adafruit_SSD1306 &disp, const char *label);

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

      selectChannel(1);
      showConnectCounterScreen(display1, title, elapsed);

      selectChannel(2);
      showLoading(display2, title, elapsed);

      selectChannel(0);
      showSubaruBootScreen(display3, elapsed);

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
  digitalWrite(A0_PIN, LOW);
  digitalWrite(A1_PIN, LOW);
  pinMode(BLUE_LED_PIN, OUTPUT);
  setBlueLed(false);
  xTaskCreatePinnedToCore(ledBlinkTask, "ledBlinkTask", 2048, nullptr, 1, nullptr, 1);
  delay(100);

  Wire.begin();
  Wire.setClock(100000);
  disableAllChannels();
  analogReadResolution(12);

  if (!initOledOnChannel(0, display1, "display 1")) {
    Serial.println("SSD1306 allocation failed for display 1");
    while (true) {
      delay(1000);
    }
  }

  if (!initOledOnChannel(1, display2, "display 2")) {
    Serial.println("SSD1306 allocation failed for display 2");
    while (true) {
      delay(1000);
    }
  }

  if (!initOledOnChannel(2, display3, "display 3")) {
    Serial.println("SSD1306 allocation failed for display 3");
    while (true) {
      delay(1000);
    }
  }

  selectChannel(1);
  display1.clearDisplay();
  display1.setTextColor(SSD1306_WHITE);
  display1.setTextSize(1);
  display1.setCursor(0, 0);
  display1.println("ESP32 OBD-II\nMAP Reader");
  display1.println("Connecting To OBD II");
  display1.display();

  selectChannel(2);
  display2.clearDisplay();
  display2.display();

  selectChannel(0);
  display3.clearDisplay();
  display3.display();

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

  delay(100);
  selectChannel(2);
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

  selectChannel(1);
  delay(50);
  Serial.println("Updating screen 0");
  showMapScreen(display1, mapKpa, mapInHg, mapPsi);

  selectChannel(2);
  delay(50);
  Serial.println("Updating screen 1");
  showStatusScreen(display2, uptime, mapKpa >= 0);

  selectChannel(0);
  delay(50);
  Serial.println("Updating screen 2");
  showAccelScreen(display3, accelX, accelY, accelZ);

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
  disp.print("SUBARU WRX");
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

