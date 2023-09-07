#include <ArduinoMqttClient.h>
#if defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_AVR_UNO_WIFI_REV2)
#include <WiFiNINA.h>
#elif defined(ARDUINO_SAMD_MKR1000)
#include <WiFi101.h>
#elif defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#elif defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#endif

// Wi-Fi credentials
char ssid[] = "lavi";
char pass[] = "laveshgarg";

// Ultrasonic sensor pins
const int trigPin = 2;
const int echoPin = 3;

// Variables for distance measurement
float duration, distance;

// Wi-Fi and MQTT client setup
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char broker[] = "broker.mqttdashboard.com";
int port = 1883;
const char topic[] = "SIT210/waves";

// Interval for sending MQTT messages
const long interval = 1000;
unsigned long previousMillis = 0;

// Counter
int count = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for serial connection
  }

  // Connect to Wi-Fi
  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(ssid);
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    Serial.print(".");
    delay(5000);
  }

  Serial.println("Connected to the network>>");
  Serial.println();

  // Connect to MQTT broker
  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection! Error code = ");
    Serial.println(mqttClient.connectError());

    while (1); // Hang if MQTT connection fails
  }

  Serial.println("You're connected ");
  Serial.println();
}

void loop() {
  mqttClient.poll();

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {

    previousMillis = currentMillis;

    // Ultrasonic sensor distance measurement
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH);
    distance = (duration * .0343) / 2;
    Serial.print("Distance: ");
    Serial.println(distance);

    // Publish MQTT message if distance is less than 12
    if (distance < 12) {
      mqttClient.beginMessage(topic);
      mqttClient.print("Rudraksh: Wave is detected, ");
      mqttClient.print("Distance: ");
      mqttClient.print(distance);
      mqttClient.endMessage();
      delay(1000);
    }

    Serial.println();

    count++;
  }
}