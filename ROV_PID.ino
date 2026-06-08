#include <Servo.h>
#include <Wire.h>
#include "MS5837.h"
Servo esc;
MS5837 sensor;
#define ESC_PIN       9
#define ESC_NEUTRAL   1500
#define MAX_DEPTH     10.0
#define WATER_DENSITY 997
#define DEADBAND      0.05
#define CONSIGNE_MIN  0.0
#define CONSIGNE_MAX  10.0
#define DEPTH_MIN    -0.5
#define DEPTH_MAX     15.0
#define PWM_RAMP_MAX  20
float Kp = 40.0, Ki = 2.0, Kd = 15.0;
float depthConsigne = 0.0;
float depthMesuree  = 0.0;
float depthOffset   = 0.0;
float erreur        = 0.0;
float erreurPrev    = 0.0;
float integrale     = 0.0;
float derivee       = 0.0;
int   valPrev       = ESC_NEUTRAL;
unsigned long lastTime     = 0;
unsigned long lastConsigne = 0;
const unsigned long SAMPLE_MS        = 50;
const unsigned long CONSIGNE_TIMEOUT = 5000;
void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(9600);
  // ESC
  esc.attach(ESC_PIN);
  esc.writeMicroseconds(ESC_NEUTRAL);
  delay(3000);
  Serial.println("ESC arme");
  // Capteur
  Wire.begin();
  while (!sensor.init()) {
    Serial.println("Capteur non detecte");
    delay(2000);
  }
  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(WATER_DENSITY);
  // Calibration offset
  sensor.read();
  depthOffset = sensor.depth();
  Serial.print("Offset : ");
  Serial.println(depthOffset);
  Serial.println("ROV pret");
  lastTime     = millis();
  lastConsigne = millis();
}
void loop() {
  unsigned long now = millis();
  if (now - lastTime < SAMPLE_MS) return;
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;
  // Reception consigne
  if (Serial1.available()) {
    String data = Serial1.readStringUntil('\n');
    float val = data.toFloat();
    if (val >= CONSIGNE_MIN && val <= CONSIGNE_MAX) {
      depthConsigne = val;
      lastConsigne  = now;
    } else {
      Serial.println("WARN: consigne aberrante ignoree");
    }
  }
  // Timeout liaison
  if (now - lastConsigne > CONSIGNE_TIMEOUT) {
    Serial.println("WARN: liaison perdue → moteur neutre");
    esc.writeMicroseconds(ESC_NEUTRAL);
    valPrev   = ESC_NEUTRAL;
    integrale = 0.0;
    return;
  } else if (now - lastConsigne > 500) {
    Serial.println("WARN: consigne non recue, derniere valeur maintenue");
  }
  // Lecture capteur
  sensor.read();
  float depth = sensor.depth() - depthOffset;
  if (depth < DEPTH_MIN || depth > DEPTH_MAX) {
    Serial.println("WARN: profondeur aberrante ignoree");
  } else {
    depthMesuree = depth;
  }
  // Envoi profondeur vers NANO
  Serial2.print(depthMesuree, 2);
  Serial2.print("\n");
  // PID
  erreur = depthConsigne - depthMesuree;
  if (abs(erreur) < DEADBAND) {
    erreur    = 0.0;
    integrale = 0.0;
  }
  integrale += erreur * dt;
  integrale  = constrain(integrale, -10.0, 10.0);
  derivee    = (erreur - erreurPrev) / dt;
  erreurPrev = erreur;
  float commande = Kp * erreur + Ki * integrale + Kd * derivee;
  int val = ESC_NEUTRAL + (int)commande;
  val = constrain(val, 1000, 2000);
  // Rampe PWM
  int delta = val - valPrev;
  delta  = constrain(delta, -PWM_RAMP_MAX, PWM_RAMP_MAX);
  val    = valPrev + delta;
  valPrev = val;
  esc.writeMicroseconds(val);
  // Debug
  Serial.print("C:"); Serial.print(depthConsigne, 2);
  Serial.print(" M:"); Serial.print(depthMesuree, 2);
  Serial.print(" E:"); Serial.print(erreur, 2);
  Serial.print(" I:"); Serial.print(integrale, 2);
  Serial.print(" D:"); Serial.print(derivee, 2);
  Serial.print(" PWM:"); Serial.println(val);
}
