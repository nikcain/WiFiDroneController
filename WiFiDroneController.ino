
#include "WiFi.h"
#include "AsyncUDP.h"
//#include <LiquidCrystal.h>

#define DEBUGOP 1

AsyncUDP udp;
IPAddress udpAddress(192,168,1,111); // address of drone
const int udpPort = 9999;

// pins
//                rs, enable, d0, d1, d2,d3
//LiquidCrystal lcd(16, 17, 5, 18, 19);
//uint8_t ledR = 32;
//uint8_t ledG = 33
//uint8_t ledB = 25;
const int LHS_Y_pin = 34;  // analog pin connected to X output
const int LHS_X_pin = 35;  // analog pin connected to Y output
const int RHS_Y_pin = 36;  // analog pin connected to X output
const int RHS_X_pin = 39;  // analog pin connected to Y output

// wifi strength indicator LEDs
/*
int latchPin = 11;      // (11) ST_CP [RCK] on 74HC595
int clockPin = 9;      // (9) SH_CP [SCK] on 74HC595
int dataPin = 12;     // (12) DS [S1] on 74HC595
*/


int dronewifistrength = 0;
int dronebatterylevel = 0;

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
      Serial.println("appinit");
      break;
    case wificonnected:
      red = 255;
      green = 255;
      blue = 0;
      Serial.println("wificonnected");
      break;
    case wifinoconnection:
      red = 255;
      green = 0;
      blue = 0;
      Serial.println("wifinoconnection");
      break;
    case packetsworking:
      red = 0;
      green = 255;
      blue = 0;
      Serial.println("packetsworking");
      break;
    case packetsblocked:
      red = 0;
      green = 0;
      blue = 255;
      Serial.println("packetsblocked");
      break;
    default:
      red = 0;
      green = 0;
      blue = 0;
      break;
  }

  //ledcWrite(ledR, 255);
  //ledcWrite(ledG, 255);
  //ledcWrite(ledB, 255);
}
#ifdef notdef
void setWiFiStrength()
{
  int rssi = WiFi.RSSI();
  int bars = 0;
  if (rssi > -90) bars = 1;
  if (rssi > -80) bars = 2;
  if (rssi > -70) bars = 3;
  if (rssi > -67) bars = 4;
  if (rssi > -30) bars = 5;

  lcd.setCursor(0,0);
  lcd.write("Ctrol WiFi: ");
  lcd.write(bars);

  lcd.setCursor(0,1);
  lcd.write("Drone WiFi: ");
  lcd.write(dronewifistrength);
/*
  byte leds =
    (controllerWiFiStrength > 0) &
    (controllerWiFiStrength > 1) >> 1 &
    (controllerWiFiStrength > 2) >> 2 &
    (controllerWiFiStrength > 3) >> 3;
    
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, LSBFIRST, leds);
  digitalWrite(latchPin, HIGH);
*/
}

#else

void setWiFiStrength()
{}
#endif

void setup() {
#ifdef DEBUGOP
  Serial.begin(115200); 
#endif

  //lcd.begin(16, 2);

  // WiFi strength LEDs
  /*
  pinMode(latchPin, OUTPUT);
  pinMode(dataPin, OUTPUT);  
  pinMode(clockPin, OUTPUT);
*/
  // Status RGB LED
  //ledcAttach(ledR, 12000, 8);  // 12 kHz PWM, 8-bit resolution
  //ledcAttach(ledG, 12000, 8);
  //ledcAttach(ledB, 12000, 8);
  
  showStatus(appinit);

  WiFi.mode(WIFI_STA);
  WiFi.begin("homewifi", "pants-run-glad");
  while (WiFi.status() != WL_CONNECTED) {
    showStatus(wifinoconnection);
    delay(500);
  }
  showStatus(wificonnected);
  udp.listen(9999);
  udp.onPacket([](AsyncUDPPacket packet) {    
    byte* b = packet.data();
    // packet should contain wifi strength and battery status of drone
    if (packet.length() == 2)
    {
      dronewifistrength = *b++;
      dronebatterylevel = *b;
    }
    
  });
}

void sendPacket(byte pitch, byte roll, byte yaw, byte throttle) {
  byte p[4];
  p[0] = roll;
  p[1] = pitch;
  p[2] = throttle;
  p[3] = yaw;


  if (!udp.connect(udpAddress, udpPort)) goto err;
  if (!udp.write(p, 4)) goto err;
  udp.close();
  showStatus(packetsworking);
  return;
  err:
    showStatus(packetsblocked);
}

void loop() {

  int lhs_x = 255*(analogRead(LHS_X_pin)/4095.0);
  int lhs_y = 255*(analogRead(LHS_Y_pin)/4095.0);
  int rhs_x = 255*(analogRead(RHS_X_pin)/4095.0);
  int rhs_y = 255*(analogRead(RHS_Y_pin)/4095.0);

  Serial.println("lhs x: " + String(lhs_x) + " y: " + String(lhs_y));
  Serial.println("rhs x: " + String(rhs_x) + " y: " + String(rhs_y));

  sendPacket(rhs_y, rhs_x, lhs_x, lhs_y);

  delay(2000);  // prob don't need this - depends on response from lots of packets
              // but maybe we need to wait for response (although the wifi and battery
              // levels won't change that often)
  setWiFiStrength();
}