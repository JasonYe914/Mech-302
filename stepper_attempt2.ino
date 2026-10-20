#include <avr/interrupt.h>
#include <AccelStepper.h>
#include <RotaryEncoder.h>

const int STEP_PIN = 9;
const int DIR_PIN = 7;
const int POLE_ENC_A = 10;
const int POLE_ENC_B = 11;
const float STEPS_PER_MM = 200.0f / 40.0f;
float distance = 3000.0f;

RotaryEncoder encoder(POLE_ENC_A, POLE_ENC_B, RotaryEncoder::LatchMode::FOUR3);
AccelStepper motor(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);

  pinMode(POLE_ENC_A, INPUT_PULLUP);
  pinMode(POLE_ENC_B, INPUT_PULLUP);

  Serial.println(F("s <vmax> <accel> | d <steps> | z"));
}

// Blocking move of `dist` steps (relative) with the given profile
void runMove(long dist, float vmax, float accel) {
  motor.setMaxSpeed(vmax);
  motor.setAcceleration(accel);
  motor.move(dist);
  while (motor.distanceToGo() != 0) {
    motor.run();
    encoder.tick();   // keep the encoder updating during the move
  }
}

void loop() {
  if (!Serial.available()) {
    return;
  }
  String input = Serial.readStringUntil('\n');
  input.trim();
  if (input.length() == 0) {
    return;
  }

  char cmd = input[0];
  String args = input.substring(1);
  args.trim();

  if (cmd == 'd') {
    distance = fabs(args.toFloat());
    if (distance == 0) {
      distance = 3000.0f;
    }
    Serial.print(F("Distance (steps): "));
    Serial.println(distance);
  }

  else if (cmd == 's') {
    int index = args.indexOf(' ');
    if (index < 0) {
      Serial.println(F("usage: s <vmax> <accel>"));
      return;
    }
    float vmax  = args.substring(0, index).toFloat();
    float accel = args.substring(index + 1).toFloat();
    long dist = (long)distance;

    Serial.print(F("Run: dist="));  Serial.print(dist);
    Serial.print(F(" vmax="));      Serial.print(vmax);
    Serial.print(F(" accel="));     Serial.println(accel);

    long startPos = motor.currentPosition();
    long encStart = encoder.getPosition();
    Serial.print(F("Start pos (steps): ")); Serial.println(startPos);
    Serial.print(F("Start encoder: "));     Serial.println(encStart);

    // Out
    unsigned long t0 = millis();
    runMove(dist, vmax, accel);
    unsigned long tout = millis() - t0;
    Serial.print(F("Out time (ms): "));   Serial.println(tout);
    Serial.print(F("Pos after out: "));   Serial.println(motor.currentPosition());
    Serial.print(F("Encoder after out: ")); Serial.println(encoder.getPosition());

    // Back
    t0 = millis();
    runMove(-dist, vmax, accel);
    unsigned long tback = millis() - t0;
    Serial.print(F("Back time (ms): "));  Serial.println(tback);

    long endPos = motor.currentPosition();
    long encEnd = encoder.getPosition();
    Serial.print(F("End pos (steps): ")); Serial.println(endPos);
    Serial.print(F("End encoder: "));     Serial.println(encEnd);
    Serial.print(F("Total time (ms): ")); Serial.println(tout + tback);
    Serial.print(F("Pos error (steps): ")); Serial.println(endPos - startPos);
    Serial.print(F("Encoder delta: "));   Serial.println(encEnd - encStart);
    Serial.println(F("Done"));
  }

  else if (cmd == 'z') {
    motor.setCurrentPosition(0);
    Serial.println(F("Zeroed"));
  }
}

