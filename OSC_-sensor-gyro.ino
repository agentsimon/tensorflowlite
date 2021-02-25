/*---------------------------------------------------------------------------------------------

  Open Sound Control (OSC) library for the ESP8266/ESP32

  Example for sending messages from the ESP8266/ESP32 to a remote computer
  The example is sending "hello, osc." to the address "/test".

  This example code is in the public domain.

  --------------------------------------------------------------------------------------------- */
#if defined(ESP8266)
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
#endif
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "Wire.h"
Adafruit_MPU6050 mpu;

char ssid[] = "simon";          // your network SSID (name)
char pass[] = "binh12345";                    // your network password


// defines variables

WiFiUDP Udp;                                // A UDP instance to let us send and receive packets over UDP
const IPAddress outIp(192, 168, 1 , 4);    // remote IP of your computer
const unsigned int outPort = 12345;          // remote port to receive OSC
const unsigned int localPort = 8888;        // local port to listen for OSC packets (actually not used for sending)

void setup() {
  Wire.begin();
  Serial.begin(115200);
  
  // Connect to WiFi network
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  Serial.println("Starting UDP");
  Udp.begin(localPort);
  Serial.print("Local port: ");
#ifdef ESP32
  Serial.println(localPort);
#else
  Serial.println(Udp.localPort());
#endif

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("");
  delay(100);
}

void loop() {
  OSCMessage msg("/gyrox");
  OSCMessage msg1("/gyroy");
  OSCMessage msg2("/gyroz");
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */

  Serial.print(g.gyro.x);
  Serial.print(",");
  Serial.print(g.gyro.y);
  Serial.print(",");
  Serial.print(g.gyro.z);
  Serial.println("");
  msg.add(g.gyro.x);
  Udp.beginPacket(outIp, outPort);
  msg.send(Udp);
  Udp.endPacket();
  msg1.empty();
  Serial.println("Message sent");
  msg1.add(g.gyro.y);
  Udp.beginPacket(outIp, outPort);
  msg1.send(Udp);
  Udp.endPacket();
  msg1.empty();
  Serial.println("Message sent");
  msg2.add(g.gyro.z);
  Udp.beginPacket(outIp, outPort);
  msg2.send(Udp);
  Udp.endPacket();
  msg2.empty();
  Serial.println("Message sent");
  delay(20);
}
