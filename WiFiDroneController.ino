
#include "WiFi.h"
#include "WiFiUdp.h"

#define DEBUGOP

WiFiUDP UDP;

const char *udpAddress = "192.168.1.111";
const int udpPort = 9999;

uint8_t ledR = 0;
uint8_t ledG = 2;
uint8_t ledB = 4;

const int LHS_X_pin = 32;  // analog pin connected to X output
const int LHS_Y_pin = 33;  // analog pin connected to Y output
const int RHS_X_pin = 34;  // analog pin connected to X output
const int RHS_Y_pin = 35;  // analog pin connected to Y output

enum appstatus {
  appinit = 0,
  wificonnected,
  wifinoconnection,
  packetsworking,
  packetsblocked
};

void showStatus(appstatus st) {
  uint32_t red;
  uint32_t green;
  uint32_t blue;

  /* TODO use RGB LED to indicate status?
    white, initialising
    yellow, wifi connection made - red no wifi
    green, packets sent & received
    blue, packets not received
  */
  switch (st) {
    case appinit:
      red = 255;
      green = 255;
      blue = 255;
      break;
    case wificonnected:
      red = 255;
      green = 255;
      blue = 0;
      break;
    case wifinoconnection:
      red = 255;
      green = 0;
      blue = 0;
      break;
    case packetsworking:
      red = 0;
      green = 255;
      blue = 0;
      break;
    case packetsblocked:
      red = 0;
      green = 0;
      blue = 255;
      break;
    default:
      red = 0;
      green = 0;
      blue = 0;
      break;
  }

  ledcWrite(ledR, 255);
  ledcWrite(ledG, 255);
  ledcWrite(ledB, 255);
}

void setup() {
#ifdef DEBUGOP
  Serial.begin(9600);
#endif
  showStatus(appinit);

  ledcAttach(ledR, 12000, 8);  // 12 kHz PWM, 8-bit resolution
  ledcAttach(ledG, 12000, 8);
  ledcAttach(ledB, 12000, 8);

  WiFi.mode(WIFI_STA);
  WiFi.begin("homewifi", "pants-run-glad");
  while (WiFi.status() != WL_CONNECTED) {
    showStatus(wifinoconnection);
    delay(500);
  }
  showStatus(wificonnected);
}

void sendPacket(int pitch, int roll, int yaw, int throttle) {
  if (!UDP.beginPacket(udpAddress, udpPort)) goto err;
  if (!UDP.printf("%c%c%c%c%c", roll, pitch, throttle, yaw)) goto err;
  if (!UDP.endPacket()) goto err;
  showStatus(packetsworking);
  return;
  err:
    showStatus(packetsblocked);
}

void loop() {
  int lhs_x = analogRead(LHS_X_pin);
  int lhs_y = analogRead(LHS_Y_pin);
  int rhs_x = analogRead(RHS_X_pin);
  int rhs_y = analogRead(RHS_Y_pin);

  sendPacket(rhs_y, rhs_x, lhs_x, lhs_y);

  delay(500);
}