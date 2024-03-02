// Feather9x_TX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (transmitter)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Feather9x_RX

#include <SPI.h>
#include <RH_RF95.h>

// Feather 32u4:
#define RFM95_CS   8
#define RFM95_RST  4
#define RFM95_INT  7

#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 868.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

void setup() {
  pinMode(RFM95_RST, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  //while (!Serial) delay(1);
  delay(100);

  Serial.println("Feather LoRa TX Test!");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
}

int16_t packetnum = 0;  // packet counter, we increment per xmission

void loop() {
  const char *messageToSend = "Puerta abierta";  // Cambia este valor según tu necesidad
  uint8_t len = strlen(messageToSend) + 1;  // Incluye el carácter nulo '\0'

  Serial.println("Sending message to server...");

  rf95.send((uint8_t *)messageToSend, len);
  
  if (rf95.waitPacketSent()) {
    Serial.println("Message sent successfully!");
    Serial.println("Waiting for ACK...");

    if (rf95.waitAvailableTimeout(3000)) {
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);
      
      if (rf95.recv(buf, &len) && len > 0 && buf[0] == 'A') {
        Serial.println("ACK received!");
        // Hacer parpadear el LED en el transmisor
        digitalWrite(LED_BUILTIN, HIGH);
        delay(500);  // Tiempo de encendido del LED (ajusta según sea necesario)
        digitalWrite(LED_BUILTIN, LOW);
      } else {
        Serial.println("Invalid or no ACK received.");
      }
    } else {
      Serial.println("No ACK received within timeout.");
    }
  } else {
    Serial.println("Error sending message. Retrying...");
  }
  delay(3000);
}
