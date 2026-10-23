#define BLYNK_TEMPLATE_ID "TMPL2W5qdfWSI"
#define BLYNK_TEMPLATE_NAME "Home"
#define BLYNK_AUTH_TOKEN "YOUR_AUTH_TOKEN"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

// Put your router Wi-Fi here.
char wifiSsid[] = "SpectrumSetup-D3";
char wifiPass[] = "finishoasis269";

const int LED_PIN = 2;
bool ledState = false;
unsigned long lastBlinkEventMs = 0;
const unsigned long EVENT_COOLDOWN_MS = 10000;

// Blynk phone button on V0.
BLYNK_WRITE(V0) {
	int value = param.asInt();
	if (value == 1) {
		digitalWrite(LED_PIN, HIGH);
		ledState = true;
		delay(200);
		digitalWrite(LED_PIN, LOW);
		ledState = false;

		unsigned long now = millis();
		if (now - lastBlinkEventMs >= EVENT_COOLDOWN_MS) {
			Blynk.logEvent("led_blink", "LED blinked from phone");
			lastBlinkEventMs = now;
		}

		Blynk.virtualWrite(V0, 0);
	}
}

void setup() {
	Serial.begin(115200);
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, LOW);

	Blynk.begin(BLYNK_AUTH_TOKEN, wifiSsid, wifiPass);
}

void loop() {
	Blynk.run();
}
