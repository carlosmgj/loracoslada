#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "MIWIFI_iq5e";
const char* password = "TdR2ddqw";
const char* mqttServer = "192.168.1.136";
const int mqttPort = 1883;
const char* mqttUser = "mqttcoslada";
const char* mqttPassword = "raspberrymqtt";


WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastmillis;

void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");

  client.setServer(mqttServer, mqttPort);

  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
    if (client.connect("ESP32Client", mqttUser, mqttPassword)) {
      Serial.println("Connected to MQTT");
    } else {
      Serial.print("Failed to connect to MQTT with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }
  lastmillis=millis();
}


void loop() {
  client.loop();

  unsigned long currentMillis = millis();

  if (currentMillis - lastmillis >= 5000) {
    // Add code here to read and publish water level
    lastmillis = currentMillis;
    client.publish("lora/test", "on");  // Replace with actual water level value
  }
}
