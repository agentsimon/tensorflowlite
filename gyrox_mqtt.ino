#include <ESP8266WiFi.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <PubSubClient.h>


Adafruit_MPU6050 mpu;

// Update these with values suitable for your network.
const char* ssid = "Jolia House"; //Replace with your SSID
const char* password = "123456789"; //Replace with your WIFI password
const char* mqtt_server = "192.168.0.26"; //Replace with the I.P of the computer
const char* mqtt_username = "MQTT username"; //Replace with the username you used in the setup of MQTT on your computer
const char* mqtt_password = "MQTT password";//Replace with the MQTT password you used when you set up MQTT on your computer
const char* clientID = "espClient";
WiFiClient wifiClient;
PubSubClient client(mqtt_server, 1883, wifiClient);
long lastMsg = 100;
// defines variables
long duration;
int distance;


void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Connect to MQTT Broker
  // client.connect returns a boolean value to let us know if the connection was successful.
  // If the connection is failing, make sure you are using the correct MQTT Username and Password (Setup Earlier in the Instructable)
  if (client.connect(clientID)) {
    Serial.println("Connected to MQTT Broker!");
  }
  else {
    Serial.println("Connection to MQTT Broker failed...");
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("arduinoClient_distance_sensor")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
 
void setup()
{
  Serial.begin(115200);
  setup_wifi(); 
  client.setServer(mqtt_server, 1883); 
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

void loop()
{
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
    sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  long now = millis();
  if (now - lastMsg > 100) {
    lastMsg = now;
     //String(g.gyro.x).c_str()
    client.publish("gyroX", String(g.gyro.x).c_str());
    client.publish("gyroY", String(g.gyro.y).c_str());
    client.publish("gyroZ", String(g.gyro.z).c_str());
    Serial.print("Distance: ");
    Serial.println(g.gyro.x);
    Serial.println(g.gyro.y);
    Serial.println(g.gyro.z);
    
  }
}
