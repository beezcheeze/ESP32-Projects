// ESP32 (classic, e.g. DEVKIT V1) talking to an OBDLink SX over its USB port.
//
// The ESP32 classic has NO native USB host controller, so it cannot enumerate
// the OBDLink SX (a USB CDC-ACM / virtual-COM device) on its own. This sketch
// requires an external MAX3421E-based "USB Host Shield" wired over SPI.
//
// Library required (Arduino Library Manager): "USB Host Shield Library 2.0"
// https://github.com/felis/USB_Host_Shield_2.0
//
// Wiring (typical USB Host Shield -> ESP32, using VSPI):
//   SCK  -> GPIO18
//   MISO -> GPIO19
//   MOSI -> GPIO23
//   SS   -> GPIO5   (change USB_SS_PIN below if you wire it differently)
//   INT  -> GPIO4   (change USB_INT_PIN below if you wire it differently)
//   VBUS/5V -> 5V, GND -> GND
//
// Protocol: the OBDLink SX speaks the same ASCII ELM327-style AT/OBD command
// set that OBDWiz uses over the COM port (e.g. "ATZ", "ATE0", "ATSP0",
// "010C" for RPM). Responses are ASCII, terminated with a "\r>" prompt.

#include <SPI.h>
#include <cdcacm.h>
#include <usbhub.h>

// Change these if your shield is wired to different pins.
#define USB_SS_PIN  5
#define USB_INT_PIN 4

class ACMAsyncOper : public CDCAsyncOper {
public:
  uint8_t OnInit(ACM *pacm) override;
};

uint8_t ACMAsyncOper::OnInit(ACM *pacm) {
  uint8_t rcode = pacm->SetControlLineState(3); // DTR=1, RTS=1
  if (rcode) {
    Serial.print("SetControlLineState failed, code ");
    Serial.println(rcode);
    return rcode;
  }

  LINE_CODING lc;
  lc.dwDTERate = 115200; // OBDLink SX default; harmless placeholder over USB CDC
  lc.bCharFormat = 0;
  lc.bParityType = 0;
  lc.bDataBits = 8;

  rcode = pacm->SetLineCoding(&lc);
  if (rcode) {
    Serial.print("SetLineCoding failed, code ");
    Serial.println(rcode);
  }
  return rcode;
}

USB Usb;
USBHub Hub(&Usb);
ACMAsyncOper AsyncOper;
ACM Acm(&Usb, &AsyncOper);

// Commands sent once at startup to reset and configure the adapter,
// then poll engine RPM (PID 0x0C) repeatedly.
const char *initCommands[] = {
  "ATZ\r",   // reset
  "ATE0\r",  // echo off
  "ATL0\r",  // linefeeds off
  "ATSP0\r"  // auto-detect OBD protocol
};
const uint8_t initCommandCount = sizeof(initCommands) / sizeof(initCommands[0]);
uint8_t initStep = 0;
unsigned long lastCommandMs = 0;
const unsigned long COMMAND_INTERVAL_MS = 300;
bool initDone = false;

void sendCommand(const char *cmd) {
  Acm.SndData(strlen(cmd), (uint8_t *)cmd);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  if (Usb.Init() == -1) {
    Serial.println("USB Host Shield init failed - check wiring");
    while (true) {}
  }
  Serial.println("USB Host Shield initialized, waiting for OBDLink SX...");
}

void loop() {
  Usb.Task();

  if (!Acm.isReady()) {
    return;
  }

  unsigned long now = millis();

  if (!initDone) {
    if (initStep < initCommandCount && now - lastCommandMs >= COMMAND_INTERVAL_MS) {
      sendCommand(initCommands[initStep]);
      initStep++;
      lastCommandMs = now;
    } else if (initStep >= initCommandCount) {
      initDone = true;
      lastCommandMs = now;
    }
  } else if (now - lastCommandMs >= COMMAND_INTERVAL_MS) {
    sendCommand("010C\r"); // request engine RPM
    lastCommandMs = now;
  }

  uint8_t buf[128];
  uint16_t rcvd = sizeof(buf);
  uint8_t rcode = Acm.RcvData(&rcvd, buf);
  if (!rcode && rcvd) {
    Serial.write(buf, rcvd);
  }
}
