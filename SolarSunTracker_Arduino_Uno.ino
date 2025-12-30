/*
  SolarSunTracker_Arduino_Uno.ino
  2-axis solar tracker for Arduino Uno using 4 LDRs and 2 hobby servos (pan + tilt).
  - Smooths LDR readings with moving average
  - Proportional control (configurable Kp) with deadband to avoid jitter
  - Night detection (light threshold) and optional park position
  - Simple serial calibration/commands to center/park/change params at runtime

  Wiring (example):
  - LDR Top-Left  -> A0
  - LDR Top-Right -> A1
  - LDR Bot-Left  -> A2
  - LDR Bot-Right -> A3
  - Servo Pan signal  -> D9
  - Servo Tilt signal -> D10
  - Servos powered from a separate 5V supply (or USB if low current), COMMON GND with Arduino
  - Each LDR used in a voltage divider: 5V -> LDR -> analog pin -> 10k -> GND
    (If you wire LDR to GND and resistor to 5V then values invert — adjust mapping or wiring consistently.)

  Serial commands (open Serial Monitor 115200):
   - 'c' : center servos to start position
   - 'p' : move to park position (parkPan, parkTilt)
   - 's' : print sensor and state status
   - 'kNNN' : set Kp_pan (e.g., 'k05' sets Kp_pan=0.05)  -> format is 'k' followed by integer (divide by 100)
   - 'dNNN' : set DEAD_BAND (ADC units) e.g., 'd20'
   - 'lNNN' : set LIGHT_THRESHOLD e.g., 'l100'
   - 'h' : print help

  Created: 2025-12-20
  Author: Copilot for alameenmamtech-maker
*/

#include <Servo.h>
#include <Arduino.h>

// ------------ PIN CONFIG ------------
const int LDR_LT_PIN = A0; // top-left
const int LDR_RT_PIN = A1; // top-right
const int LDR_LB_PIN = A2; // bottom-left
const int LDR_RB_PIN = A3; // bottom-right

const int SERVO_PAN_PIN  = 9;  // horizontal (azimuth)
const int SERVO_TILT_PIN = 10; // vertical (elevation)

// ------------ SERVO / MOVEMENT SETTINGS ------------
Servo servoPan;
Servo servoTilt;

// initial positions (degrees)
int panPos  = 90; // middle
int tiltPos = 90;

// mechanical limits (adjust per mounting)
const int SERVO_MIN = 0;
const int SERVO_MAX = 180;

// Park position when sun is down
const int parkPan  = 90;
const int parkTilt = 20;

// Center (startup) position
const int centerPan  = 90;
const int centerTilt = 90;

// ------------ CONTROL / SENSOR SETTINGS ------------
float Kp_pan  = 0.05; // proportional gain for pan. Tune (0.03..0.2)
float Kp_tilt = 0.05; // proportional gain for tilt.

int DEAD_BAND = 20;        // difference threshold in ADC units to ignore micro-errors
int LIGHT_THRESHOLD = 80;  // minimum average ADC reading to consider it's daylight (0..1023)

const unsigned long LOOP_DELAY_MS = 150; // control loop delay

// ------------ SMOOTHING (moving average) ------------
const int SMOOTH_N = 6; // moving-average window
int bufLT[SMOOTH_N], bufRT[SMOOTH_N], bufLB[SMOOTH_N], bufRB[SMOOTH_N];
int bufIndex = 0;

// ------------ STATE ------------
bool isParked = false;

// ------------ SETUP ------------
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; } // wait for serial on some boards (safe)

  servoPan.attach(SERVO_PAN_PIN);
  servoTilt.attach(SERVO_TILT_PIN);

  panPos = centerPan;
  tiltPos = centerTilt;
  servoPan.write(panPos);
  servoTilt.write(tiltPos);

  // initialize smoothing buffers with current readings
  int initVal = analogRead(LDR_LT_PIN);
  for (int i = 0; i < SMOOTH_N; ++i) {
    bufLT[i] = bufRT[i] = bufLB[i] = bufRB[i] = initVal;
  }

  Serial.println(F("Solar Sun Tracker (Arduino Uno)"));
  printHelp();
  delay(1000);
}

// ------------ HELPERS ------------
int smoothRead(int buf[], int pin) {
  buf[bufIndex] = analogRead(pin);
  long sum = 0;
  for (int i = 0; i < SMOOTH_N; ++i) sum += buf[i];
  return (int)(sum / SMOOTH_N);
}

void printStatus(int lt, int rt, int lb, int rb, int avg, int herr, int verr) {
  Serial.print("LT:"); Serial.print(lt);
  Serial.print(" RT:"); Serial.print(rt);
  Serial.print(" LB:"); Serial.print(lb);
  Serial.print(" RB:"); Serial.print(rb);
  Serial.print(" AVG:"); Serial.print(avg);
  Serial.print(" Herr:"); Serial.print(herr);
  Serial.print(" Verr:"); Serial.print(verr);
  Serial.print(" Pan:"); Serial.print(panPos);
  Serial.print(" Tilt:"); Serial.println(tiltPos);
}

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  c     -> center servos"));
  Serial.println(F("  p     -> park servos"));
  Serial.println(F("  s     -> status"));
  Serial.println(F("  kNNN  -> set Kp_pan & Kp_tilt (integer, divided by 100). e.g. k05 -> 0.05"));
  Serial.println(F("  dNNN  -> set DEAD_BAND (ADC units)"));
  Serial.println(F("  lNNN  -> set LIGHT_THRESHOLD (ADC units)"));
  Serial.println(F("  h     -> help"));
}

// ------------ LOOP ------------
void loop() {
  // read smoothed sensors
  int lt = smoothRead(bufLT, LDR_LT_PIN);
  int rt = smoothRead(bufRT, LDR_RT_PIN);
  int lb = smoothRead(bufLB, LDR_LB_PIN);
  int rb = smoothRead(bufRB, LDR_RB_PIN);

  bufIndex = (bufIndex + 1) % SMOOTH_N;
  int avgLight = (lt + rt + lb + rb) / 4;

  // check for serial commands (non-blocking)
  handleSerialCommands();

  // night detection
  if (avgLight < LIGHT_THRESHOLD) {
    if (!isParked) {
      Serial.println(F("Low light detected -> parking to conserve/avoid hunting."));
      park();
      isParked = true;
    }
    delay(LOOP_DELAY_MS);
    return;
  } else {
    if (isParked) {
      Serial.println(F("Light returned -> resuming tracking."));
      // Optionally restore to center or previous pos
      isParked = false;
    }
  }

  // compute left/right & top/bottom totals
  int leftTotal  = lt + lb;
  int rightTotal = rt + rb;
  int topTotal   = lt + rt;
  int botTotal   = lb + rb;

  int horizError = leftTotal - rightTotal; // >0 => sun toward left
  int vertError  = topTotal - botTotal;    // >0 => sun toward top

  // deadband
  if (abs(horizError) < DEAD_BAND) horizError = 0;
  if (abs(vertError)  < DEAD_BAND) vertError  = 0;

  // map proportional error to servo delta (degrees)
  float deltaPan  = Kp_pan * (float)horizError;
  float deltaTilt = Kp_tilt * (float)vertError;

  // adjust sign depending on mounting; test and invert if movement is opposite
  // Here: positive horizError means sun on left, so decrease pan angle to move left (servo mapping may vary)
  panPos  = constrain(panPos - (int)round(deltaPan), SERVO_MIN, SERVO_MAX);
  tiltPos = constrain(tiltPos + (int)round(deltaTilt), SERVO_MIN, SERVO_MAX);

  servoPan.write(panPos);
  servoTilt.write(tiltPos);

  // debug/status print occasionally (or on demand)
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 1000) {
    printStatus(lt, rt, lb, rb, avgLight, horizError, vertError);
    lastPrint = millis();
  }

  delay(LOOP_DELAY_MS);
}

// ------------ PARK / CENTER ------------
void park() {
  panPos = parkPan;
  tiltPos = parkTilt;
  servoPan.write(panPos);
  servoTilt.write(tiltPos);
  Serial.print("Parked: pan="); Serial.print(panPos); Serial.print(" tilt="); Serial.println(tiltPos);
}

void center() {
  panPos = centerPan;
  tiltPos = centerTilt;
  servoPan.write(panPos);
  servoTilt.write(tiltPos);
  Serial.print("Centered: pan="); Serial.print(panPos); Serial.print(" tilt="); Serial.println(tiltPos);
}

// ------------ SERIAL COMMAND HANDLER ------------
void handleSerialCommands() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;

  char c = cmd.charAt(0);
  if (c == 'c' || c == 'C') {
    center();
  } else if (c == 'p' || c == 'P') {
    park();
    isParked = true;
  } else if (c == 's' || c == 'S') {
    Serial.println(F("Status:"));
    Serial.print("Kp_pan="); Serial.print(Kp_pan, 3);
    Serial.print(" Kp_tilt="); Serial.print(Kp_tilt, 3);
    Serial.print(" DEAD_BAND="); Serial.print(DEAD_BAND);
    Serial.print(" LIGHT_THRESHOLD="); Serial.println(LIGHT_THRESHOLD);
    // Show one immediate reading
    int lt = analogRead(LDR_LT_PIN), rt = analogRead(LDR_RT_PIN), lb = analogRead(LDR_LB_PIN), rb = analogRead(LDR_RB_PIN);
    int avg = (lt + rt + lb + rb) / 4;
    printStatus(lt, rt, lb, rb, avg, (lt+lb)-(rt+rb), (lt+rt)-(lb+rb));
  } else if (c == 'k' || c == 'K') {
    // expect integer after letter; convert to float by dividing by 100
    String num = cmd.substring(1);
    int v = num.toInt();
    if (v > 0) {
      float newK = v / 100.0;
      Kp_pan = newK;
      Kp_tilt = newK;
      Serial.print("Set Kp_pan & Kp_tilt = ");
      Serial.println(newK, 3);
    } else {
      Serial.println("Invalid Kp format. Example: k05 sets Kp=0.05");
    }
  } else if (c == 'd' || c == 'D') {
    String num = cmd.substring(1);
    int v = num.toInt();
    if (v >= 0) {
      DEAD_BAND = v;
      Serial.print("Set DEAD_BAND = "); Serial.println(DEAD_BAND);
    } else Serial.println("Invalid DEAD_BAND value.");
  } else if (c == 'l' || c == 'L') {
    String num = cmd.substring(1);
    int v = num.toInt();
    if (v >= 0) {
      LIGHT_THRESHOLD = v;
      Serial.print("Set LIGHT_THRESHOLD = "); Serial.println(LIGHT_THRESHOLD);
    } else Serial.println("Invalid LIGHT_THRESHOLD value.");
  } else if (c == 'h' || c == 'H') {
    printHelp();
  } else {
    Serial.print("Unknown command: "); Serial.println(cmd);
    printHelp();
  }
}