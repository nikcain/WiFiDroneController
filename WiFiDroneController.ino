
#include "WiFi.h"
#include "WiFiUdp.h"

WiFiUDP UDP;

const int SW_pin = 2;  // digital pin connected to switch output
const int X_pin = A0;  // analog pin connected to X output
const int Y_pin = A1;  // analog pin connected to Y output

void setup() {
  pinMode(SW_pin, INPUT);
  digitalWrite(SW_pin, HIGH);
  Serial.begin(9600);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin("homewifi", "pants-run-glad");
  while (WiFi.status() != WL_CONNECTED) { delay(500); }
  Serial.println(WiFi.localIP());
}

void sendPacket(int roll, int pitch, int throttle)
{
  char packet[4];

  UDP.begin(9999);
  packet[0] = roll;  
  packet[1] = pitch;
  packet[2] = throttle;
  packet[3] = yaw;
  
  UDP.beginPacket()
}

void loop() {
  Serial.print("Switch:  ");
  Serial.print(digitalRead(SW_pin));
  Serial.print("\n");
  Serial.print("X-axis: ");
  Serial.print(analogRead(X_pin));
  Serial.print("\n");
  Serial.print("Y-axis: ");
  Serial.println(analogRead(Y_pin));
  Serial.print("\n\n");
  delay(500);
}