#include <AccelStepper.h>
#include <RotaryEncoder.h>

const int STEP_PIN   = 9;
const int DIR_PIN    = 7;
const int POLE_ENC_A = 10;
const int POLE_ENC_B = 11;

const float MICROSTEPS   = 1.0f;
const float MOTOR_STEPS  = 200.0f;                 // 1.8 deg motor
const float MM_PER_REV   = 40.0f;                  // GT2 (2 mm pitch) x 20T pulley
const float STEPS_PER_MM = MOTOR_STEPS * MICROSTEPS / MM_PER_REV;
const float MM_PER_STEP  = 1.0f / STEPS_PER_MM;

const float MIN_DIST_MM   = 50.0f;
const float MAX_DIST_MM   = 300.0f;
const float MARK_EVERY_MM = 10.0f;
const unsigned long SAMPLE_US = 5000UL;            // 200 Hz

float distance_mm = 1500.0f;

RotaryEncoder encoder(POLE_ENC_A, POLE_ENC_B, RotaryEncoder::LatchMode::FOUR3);
AccelStepper  motor(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

long mmToSteps(float mm) { return lroundf(mm * STEPS_PER_MM); }

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);

  pinMode(POLE_ENC_A, INPUT_PULLUP);
  pinMode(POLE_ENC_B, INPUT_PULLUP);

  motor.setMinPulseWidth(2);   

  Serial.print(F("# steps/mm = ")); Serial.println(STEPS_PER_MM);
  Serial.println(F("# s <vmax mm/s> <accel mm/s^2> | d <mm 50-300> | a <vmax> <accel> (sweep 50..300) | z"));
}

void logLine(char tag, unsigned long t_ms, long posSteps, long enc) {
  Serial.print(tag);                        Serial.print(',');
  Serial.print(t_ms);                       Serial.print(',');
  Serial.print(posSteps * MM_PER_STEP, 1);  Serial.print(',');
  Serial.println(enc);
}

unsigned int runMove(float dist_mm, float vmax_mm_s, float accel_mm_s2,
                     long origin, unsigned long tRun) {
  motor.setMaxSpeed(vmax_mm_s);        // set limits BEFORE move()
  motor.setAcceleration(accel_mm_s2);
  motor.move(mmToSteps(dist_mm));

  const long legStart  = motor.currentPosition();
  const long markSteps = mmToSteps(MARK_EVERY_MM);
  long nextMark = markSteps;

  unsigned long nextSample = micros();
  unsigned int dropped = 0;

  while (motor.distanceToGo() != 0) {
    motor.run();
    encoder.tick();                                   // poll encoder every pass

    const unsigned long now = micros();
    const long pos       = motor.currentPosition() - origin;
    const long travelled = labs(motor.currentPosition() - legStart);
    const long enc       = encoder.getPosition();

    if (travelled >= nextMark) {                      // every 10 mm of this leg
      logLine('M', (now - tRun) / 1000UL, pos, enc);
      nextMark += markSteps;
    }
    if ((long)(now - nextSample) >= 0) {              // every 5 ms
      nextSample += SAMPLE_US;
      if (Serial.availableForWrite() >= 24) logLine('S', (now - tRun) / 1000UL, pos, enc);
      else dropped++;
    }
  }
  return dropped;
}

void doRun(float dist_mm, float vmax, float accel) {
  const long origin   = motor.currentPosition();
  const long encStart = encoder.getPosition();

  Serial.print(F("# run dist_mm=")); Serial.print(dist_mm);
  Serial.print(F(" vmax=")); Serial.print(vmax);
  Serial.print(F(" accel=")); Serial.print(accel);
  Serial.print(F(" enc0=")); Serial.println(encStart);
  Serial.println(F("tag,t_ms,pos_mm,enc"));

  const unsigned long tRun = micros();
  unsigned int dropped = runMove( dist_mm, vmax, accel, origin, tRun);
  const unsigned long tOut = (micros() - tRun) / 1000UL;
  dropped += runMove(-dist_mm, vmax, accel, origin, tRun);
  const unsigned long tTotal = (micros() - tRun) / 1000UL;

  Serial.print(F("# out_ms="));   Serial.print(tOut);
  Serial.print(F(" back_ms="));   Serial.print(tTotal - tOut);
  Serial.print(F(" total_ms=")); Serial.println(tTotal);
  Serial.print(F("# pos_error_steps=")); Serial.print(motor.currentPosition() - origin);
  Serial.print(F(" enc_delta="));       Serial.println(encoder.getPosition() - encStart);
  if (dropped) { Serial.print(F("# dropped_samples=")); Serial.println(dropped); }
  Serial.println(F("# done"));
}

bool parseTwoFloats(const String& args, float& a, float& b) {
  int sp = args.indexOf(' ');
  if (sp < 0) return false;
  a = args.substring(0, sp).toFloat();
  b = args.substring(sp + 1).toFloat();
  return (a > 0 && b > 0);
}

void loop() {
  if (!Serial.available()) return;
  String input = Serial.readStringUntil('\n');
  input.trim();
  if (input.length() == 0) return;

  char   cmd  = input[0];
  String args = input.substring(1);
  args.trim();

  if (cmd == 'd') {
    float d = fabs(args.toFloat());
    if (d < MIN_DIST_MM || d > MAX_DIST_MM) {
      Serial.println(F("# distance must be 50-300 mm"));
      return;
    }
    distance_mm = d;
    Serial.print(F("# distance = ")); Serial.print(distance_mm);
    Serial.print(F(" mm = ")); Serial.print(mmToSteps(distance_mm)); Serial.println(F(" steps"));
  }

  else if (cmd == 's') {                               // single run at current distance
    float vmax, accel;
    if (!parseTwoFloats(args, vmax, accel)) {
      Serial.println(F("# usage: s <vmax mm/s> <accel mm/s^2>"));
      return;
    }
    doRun(distance_mm, vmax, accel);
  }

  else if (cmd == 'a') {                              
    float vmax, accel;
    if (!parseTwoFloats(args, vmax, accel)) {
      Serial.println(F("# usage: a <vmax mm/s> <accel mm/s^2>"));
      return;
    }
    for (float d = MIN_DIST_MM; d <= MAX_DIST_MM + 0.01f; d += MARK_EVERY_MM) {
      doRun(d, vmax, accel);
    }
  }

  else if (cmd == 'z') {
    motor.setCurrentPosition(0);
    encoder.setPosition(0);
    Serial.println(F("# zeroed (motor + encoder)"));
  }
}