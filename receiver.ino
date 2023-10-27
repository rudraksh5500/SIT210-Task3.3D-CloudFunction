#include <ArduinoMqttClient>

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

char ssid[] = "lavi";    // Your Wi-Fi SSID
char pass[] = "laveshgarg";  // Your Wi-Fi password

int light = 2;  // Pin for controlling the light

WiFiClient wifiClient;  // Initialize a Wi-Fi client
MqttClient mqttClient(wifiClient);  // Initialize an MQTT client

const char broker[] = "broker.mqttdashboard.com";  // MQTT broker address
int port = 1883;  // MQTT broker port
const char topic[] = "SIT210/waves";  // MQTT topic to subscribe to

void setup() {
  Serial.begin(9600);  // Initialize serial communication
  pinMode(light, OUTPUT);  // Set the light pin as an output
  while (!Serial) {
    ; // Wait for serial connection to be established
  }

  Serial.print("Connecting to Wi-Fi... ");
  Serial.println(ssid);
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    Serial.print(".");
    delay(5000);
  }

  Serial.println("Connected to the network");
  Serial.println();

  Serial.print("Connecting to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    while (1); // Hang if MQTT connection fails
  }

  Serial.println("Connected to the MQTT broker!");
  Serial.println();

  Serial.print("Subscribing to the MQTT topic: ");
  Serial.println(topic);
  Serial.println();

  mqttClient.subscribe(topic);

  Serial.print("Waiting for messages on topic: ");
  Serial.println(topic);
  Serial.println();
}

void loop() {
  int messageSize = mqttClient.parseMessage();
  if (messageSize) {
    Serial.print("Received a message with topic '");
    Serial.print(mqttClient.messageTopic());
    Serial.print("', length ");
    Serial.print(messageSize);
    Serial.println(" bytes:");

    while (mqttClient.available()) {
      Serial.print((char)mqttClient.read());
    }
    Serial.println();

    // Blink the light to indicate message reception
    digitalWrite(light, HIGH);
    delay(200);
    digitalWrite(light, LOW);
    delay(200);
    digitalWrite(light, HIGH);
    delay(200);
    digitalWrite(light, LOW);
    delay(200);
    digitalWrite(light, HIGH);
    delay(200);
    digitalWrite(light, LOW);
    delay(200);

    Serial.println();
  }
}
