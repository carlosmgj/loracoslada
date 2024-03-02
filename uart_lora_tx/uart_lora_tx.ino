#include <SoftwareSerial.h>

SoftwareSerial mySerial(11, 12);
int i=0;
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.begin(115200);
  mySerial.begin(9600);
}

void loop() {
  i++;
  String dataToSend = "Hooola numero "+String(i)+"\n";
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Enviando: " + dataToSend);
  mySerial.print(dataToSend);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);

  // Esperar la confirmación del ESP32
  if (waitForConfirmation()) {
    Serial.println("Confirmación recibida. Datos enviados correctamente.");
  } else {
    Serial.println("Error: No se recibió confirmación. Reenviando datos...");
    // Puedes implementar un mecanismo de reintento aquí
    // Por ejemplo, un bucle que reintente el envío varias veces antes de abortar
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
