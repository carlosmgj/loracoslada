#include <SPI.h>
#include <RH_RF95.h>
#include <SoftwareSerial.h>

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7
#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2
#define RF95_FREQ 868.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);
SoftwareSerial mySerial(11, 12);
int i = 0;

const int maxMessageLength = 64;  // Ajusta según la longitud máxima del mensaje
char receivedMessage[maxMessageLength];

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  Serial.begin(115200);
  mySerial.begin(9600);
  delay(100);
  Serial.println("Feather LoRa RX Test!");

  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1)
      ;
  }
  Serial.println("LoRa radio init OK!");

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1)
      ;
  }
  Serial.print("Set Freq to: ");
  Serial.println(RF95_FREQ);
  rf95.setTxPower(23, false);
}

void loop() {
  if (rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
      Serial.print("Received message from client: ");
      
      // Almacenar el mensaje en el array de caracteres
      for (int i = 0; i < len && i < maxMessageLength - 1; i++) {
        receivedMessage[i] = (char)buf[i];
        Serial.print(receivedMessage[i]);
      }
      receivedMessage[len] = '\0';  // Asegurarse de que el array de caracteres termine con '\0' (nulo)
      
      Serial.println();

      uint8_t senderAddress = rf95.headerFrom();
      int16_t rssi = rf95.lastRssi();
      Serial.print("RSSI: ");
      Serial.print(rssi);
      Serial.println(" dBm");

      digitalWrite(LED_BUILTIN, HIGH);
      delay(500);
      digitalWrite(LED_BUILTIN, LOW);

      Serial.println("Sending data to UART...");

      i++;
      String dataToSend = String(receivedMessage)+"\n";
      digitalWrite(LED_BUILTIN, HIGH);
      Serial.println("Enviando: " + dataToSend);
      mySerial.print(dataToSend);

      Serial.println("Data sent to UART successfully!");
      if (waitForConfirmation()) {
        Serial.println("Confirmación recibida. Datos enviados correctamente.");
      } else {
        Serial.println("Error: No se recibió confirmación. Reenviando datos...");
        // Puedes implementar un mecanismo de reintento aquí
        // Por ejemplo, un bucle que reintente el envío varias veces antes de abortar
      }
    } else {
      Serial.println("Error receiving message.");
    }
  }
}

bool waitForConfirmation() {
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {  // Esperar hasta 5 segundos para la confirmación
    if (mySerial.available()) {
      char confirmationChar = mySerial.read();
      return (confirmationChar == 'C');  // Ajusta el carácter de confirmación según tus necesidades
    }
  }
  return false;  // No se recibió confirmación dentro del tiempo límite
}
