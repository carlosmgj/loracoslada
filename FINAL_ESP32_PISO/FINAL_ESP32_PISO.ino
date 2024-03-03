#include <WiFi.h>
#include <PubSubClient.h>

#define RXD2 16
#define TXD2 17

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
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

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

  lastmillis = millis();
}

void loop() {
  client.loop();

  unsigned long currentMillis = millis();

  if (currentMillis - lastmillis >= 5000) {
    lastmillis = currentMillis;
    //client.publish("lora/test", "on");  // Reemplaza con la lógica de publicación real
  }

  static String receivedString = "";

  while (Serial2.available()) {
    char receivedChar = Serial2.read();

    if (receivedChar == '\n') {
      processString(receivedString);
      Serial2.print('C');
      Serial2.flush();
      receivedString = "";
    } else {
      receivedString += receivedChar;
    }
  }
}

void processString(String data) {
  Serial.println("Nuevo string recibido: " + data);

  // Analizar el mensaje y publicar en MQTT según sea necesario
  if (data.indexOf("HeartBeat") != -1) {
    client.publish("lora/test", "off");
    Serial.println("Publicando 'off' en lora/test");
  } else if (data.indexOf("CAMBIO") != -1 || data.indexOf("ALERTA") != -1) {
    client.publish("lora/test", "on");
    Serial.println("Publicando 'on' en lora/test");
  } else
  {
    Serial.println("error analizando el mensaje");
  }
}
