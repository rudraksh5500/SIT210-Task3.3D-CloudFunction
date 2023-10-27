#include <ArduinoMqttClient.h>
// Include the appropriate Wi-Fi library based on the board type
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
char ssid[] = "lavi"; // Your Wi-Fi SSID
char pass[] = "laveshgarg"; // Your Wi-Fi password

// Ultrasonic sensor pins
const int trigPin = 2; // Trigger pin of the ultrasonic sensor
const int echoPin = 3; // Echo pin of the ultrasonic sensor

// Variables for distance measurement
float duration, distance;

// Wi-Fi and MQTT client setup
WiFiClient wifiClient; // Initialize a Wi-Fi client
MqttClient mqttClient(wifiClient); // Initialize an MQTT client

const char broker[] = "broker.mqttdashboard.com"; // MQTT broker address
int port = 1883; // MQTT broker port
const char topic[] = "SIT210/waves"; // MQTT topic to publish to

// Interval for sending MQTT messages
const long interval = 1000; // Time interval in milliseconds
unsigned long previousMillis = 0;

// Counter
int count = 0;

void setup() {
  Serial.begin(9600); // Initialize serial communication
  while (!Serial) {
    ; // Wait for serial connection to be established
  }

  // Connect to Wi-Fi
  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(ssid);
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    Serial.print(".");
    delay(5000);
  }

  Serial.println("Connected to the network");
  Serial.println();

  // Connect to MQTT broker
  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection! Error code = ");
    Serial.println(mqttClient.connectError());

    while (1); // Hang if MQTT connection fails
  }

  Serial.println("You're connected to MQTT");
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
    distance = (duration * 0.0343) / 2; // Calculate distance based on the duration
    Serial.print("Distance: ");
    Serial.println(distance);

    // Publish MQTT message if distance is less than 12
    if (distance < 12) {
      mqttClient.beginMessage(topic);
      mqttClient.print("Rudraksh: Wave is detected, ");
      mqttClient.print("Distance: ");
      mqttClient.print(distance);
      mqttClient.endMessage();
      delay(1000); // Delay before sending the next message
    }

    Serial.println();

    count++;
  }
}
