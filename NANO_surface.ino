#include <SoftwareSerial.h>
SoftwareSerial RS485(7, 4);
#define POT_PIN   A0
#define MAX_DEPTH 10.0
void setup() {
  Serial.begin(9600);
  RS485.begin(9600);
  Serial.println("Surface prete");
}
void loop() {
  int potValue = analogRead(POT_PIN);
  float consigne = (potValue / 1023.0) * MAX_DEPTH;
  RS485.print(consigne, 2);
  RS485.print("\n");
  if (RS485.available()) {
    String data = RS485.readStringUntil('\n');
    float profondeurReelle = data.toFloat();
    Serial.print("Consigne : ");
    Serial.print(consigne, 2);
    Serial.print(" m | Profondeur ROV : ");
    Serial.print(profondeurReelle, 2);
    Serial.println(" m");
  }
  delay(100);
}
