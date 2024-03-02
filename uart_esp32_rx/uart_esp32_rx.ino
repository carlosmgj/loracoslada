#define RXD2 16
#define TXD2 17

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial.println("Serial Txd is on pin: " + String(TX));
  Serial.println("Serial Rxd is on pin: " + String(RX));
}

void loop() {
  static String receivedString = "";

  while (Serial2.available()) {
    char receivedChar = Serial2.read();

    if (receivedChar == '\n') {
      // Fin del string, procesar el string recibido
      processString(receivedString);

      // Enviar confirmación al ATmega32U4
      Serial2.print('C');
      Serial2.flush();  // Asegurar que la confirmación se envíe de inmediato

      // Reiniciar el string para la próxima transmisión
      receivedString = "";
    } else {
      // Agregar el carácter al string
      receivedString += receivedChar;
    }
  }
}

void processString(String data) {
  Serial.println("Nuevo string recibido: " + data);
}
