// @file FINAL_LORA_PISO.ino
/* 
Código para el Lora32u4 ubicado cerca del router. 
 Características:
 - Conectado por UART a ESP32 (éste se encarga del volcado de datos al servidor, en este caso HA).
 - Recibe datos por red LoRa. Nodos:
  - Nodo 1: este MCU
  - Nodo 2: ubicado en posición intermedia, con alcance a los otros dos nodos.
  - Nodo 3: ubicado en la posición más alejada a este nodo. Utilizado actualmente para el coche. 
  - Campos del mensaje:
    - Nodo 1 <- Nodo 2
      - Sensores del nodo 2:
        - Puerta del trastero:
          - Nombre: PuertaTrastero
          - Valores: on/off
        - Nivel de batería (Opcional futuro)
          - TrasteroBateria
          - Valores: 0-100
      - Sensores del nodo 3:
        - Puerta del coche:
          - Nombre: PuertaCoche
          - Valores: on/off
        - ...resto de sensores del coche
    - Nodo 2 <- Nodo 3
      - Mismos sensores listados arriba bajo "Sensores del nodo 3"
- Funcionamiento específico de este nodo:
  - Timer de (TBD) segundos que cuando llega a 0, lanza una interrupción a un callback llamado "WatchdogCallback"
  - Recepción de mensajes de LoRa + ACK a remitente
  - Envío del mensaje de LoRa sin analizar al ESP32 por UART (SoftwareSerial).
- Nomenclatura mensajes:
  - {"key":"puerta1","value":"on/off"},{"key":"sensorX","value":"0-100,on/off"}
*/

#include <SPI.h>
#include <RH_RF95.h>
#include <SoftwareSerial.h>

// Definicion de parametros para comunicación ATMEGA32u4 <-> RFM95 (LoRa)

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7
#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2
#define RF95_FREQ 868.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Definicion de parametros para comunicación ESP32 <-> ATMEGA32u4.
// mySerial(RX,TX)

SoftwareSerial mySerial(11, 12);
#define USEDEBUGSERIAL 1

const int maxMessageLength = 64;
char receivedMessage[maxMessageLength];


unsigned long startMillis;
unsigned long currentMillis;
const unsigned long period = 20000;  //ms


void setup() {

  //Declaración pin LED NATIVO e inicialización:
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  //Declaración pin RESET de RFM95 e inicialización
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // Comunicaciones Serie. (Serial para USB y mySerial para ESP32)
  if (USEDEBUGSERIAL)
    Serial.begin(115200);
  mySerial.begin(9600);
  delay(100);
  if (USEDEBUGSERIAL)
    Serial.println("Feather LoRa RX Test!");

  // Reseteo manual de RFM95
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    if (USEDEBUGSERIAL) {
      Serial.println("LoRa radio init failed");
      Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    }
    while (1)
      ;
  }
  if (USEDEBUGSERIAL)
    Serial.println("LoRa radio init OK!");

  if (!rf95.setFrequency(RF95_FREQ)) {
    if (USEDEBUGSERIAL)
      Serial.println("setFrequency failed");
    while (1)
      ;
  }
  if (USEDEBUGSERIAL) {
    Serial.print("Set Freq to: ");
    Serial.println(RF95_FREQ);
  }
  rf95.setTxPower(23, false);

  startMillis = millis();  //initial start time
}

void loop() {

  // Si llega un mensaje de Lora....
  if (rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    //Almaceno el mensaje en un array
    if (rf95.recv(buf, &len)) {
      if (USEDEBUGSERIAL)
        Serial.print("Received message from client: ");
      // Almacenar el mensaje en el array de caracteres
      for (int i = 0; i < len && i < maxMessageLength - 1; i++) {
        receivedMessage[i] = (char)buf[i];
        if (USEDEBUGSERIAL)
          Serial.print(receivedMessage[i]);
      }
      // REINICIAMOS EL CONTADOR DE COMMLOST
      startMillis = millis();

      receivedMessage[len] = '\0';  // Asegurarse de que el array de caracteres termine con '\0' (nulo)
      if (USEDEBUGSERIAL)
        Serial.println();

      uint8_t senderAddress = rf95.headerFrom();
      int16_t rssi = rf95.lastRssi();
      if (USEDEBUGSERIAL) {
        Serial.print("RSSI: ");
        Serial.print(rssi);
        Serial.println(" dBm");
        Serial.println("Sending data to UART...");
      }

      if (USEDEBUGSERIAL)
        Serial.println("Sending ACK to client...");

      rf95.setHeaderTo(senderAddress);
      // Send the ACK message
      const char* ackMessage = "ACK";
      rf95.send((uint8_t*)ackMessage, strlen(ackMessage) + 1);
      rf95.waitPacketSent();

      String dataToSend = "Sender:" + String(senderAddress) + "-RSSI:" + String(rssi) + "-" + String(receivedMessage) + "\n";
      if (USEDEBUGSERIAL)
        Serial.println("Enviando: " + dataToSend);
      mySerial.print(dataToSend);
      if (USEDEBUGSERIAL)
        Serial.println("Data sent to UART successfully!");
      if (waitForConfirmation()) {
        if (USEDEBUGSERIAL)
          Serial.println("Confirmación recibida. Datos enviados correctamente.");
      } else {
        if (USEDEBUGSERIAL)
          Serial.println("Error: No se recibió confirmación. Reenviando datos...");
        // Puedes implementar un mecanismo de reintento aquí
        // Por ejemplo, un bucle que reintente el envío varias veces antes de abortar
      }
    } else {
      if (USEDEBUGSERIAL)
        Serial.println("Error receiving message.");
    }
  }
  currentMillis = millis();
  if (currentMillis - startMillis >= period) {
    startMillis = millis();  //initial start time
    if (USEDEBUGSERIAL)
      Serial.println("Perdida comunicacion con nodo 1");
    String dataToSend2 = "CommLost \n";
    mySerial.print(dataToSend2);
    if (waitForConfirmation()) {
      if (USEDEBUGSERIAL)
        Serial.println("Confirmación recibida. Datos enviados correctamente.");
    }else
    {
      if (USEDEBUGSERIAL)
        Serial.println("ERROR");
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
