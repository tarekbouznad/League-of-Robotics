/*
PID Motor Control for Differential Drive
----------------------------------------
This code implements a PID control system to adjust motor speeds based on error values.
*/

#define r1 2  // Motor Right IN1
#define r2 4  // Motor Right IN2
#define enR 3 // Motor Right Enable (PWM)
#define enL 5 // Motor Left Enable (PWM)
#define l1 9  // Motor Left IN3
#define l2 8  // Motor Left IN4

// PID Control Parameters
#define KP 0.25
#define KD 5
#define SET_SPEED 50
#define MAX_SPEED 100
#define MIN_SPEED 0

// Variables for PID calculation
int lastError = 0;        // Stores the previous error
int erreur = 0;           // Current error value
int power_difference = 0; // Power difference between motors
int motorSpeeds[2] = {0, 0}; // Array for left and right motor speeds

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Configure motor control pins as outputs
  pinMode(r1, OUTPUT);
  pinMode(r2, OUTPUT);
  pinMode(l1, OUTPUT);
  pinMode(l2, OUTPUT);
  pinMode(enL, OUTPUT);
  pinMode(enR, OUTPUT);
}

void loop() {
  // Check for serial input
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n'); // Read error value from serial
    erreur = data.toInt();                     // Convert to integer

    // Execute PID control
    pidControl();

    // Debugging: Print motor speeds
    Serial.print("Left Motor Speed: ");
    Serial.print(motorSpeeds[0]);
    Serial.print(", Right Motor Speed: ");
    Serial.println(motorSpeeds[1]);
  }
}

// Function to execute PID control
void pidControl() {
  // Calculate derivative term
  int d = erreur - lastError;
  lastError = erreur;

  // Compute power difference using PID formula
  power_difference = erreur * KP + d * KD;

  // Constrain power difference to prevent over-speeding
  power_difference = constrain(power_difference, -50, 50);

  // Calculate motor speeds
  motorSpeeds[0] = constrain(SET_SPEED + power_difference, MIN_SPEED, MAX_SPEED); // Left motor
  motorSpeeds[1] = constrain(SET_SPEED - power_difference, MIN_SPEED, MAX_SPEED); // Right motor

  // Drive the motors
  digitalWrite(l1, HIGH);
  digitalWrite(l2, LOW);
  digitalWrite(r1, LOW);
  digitalWrite(r2, HIGH);
  analogWrite(enL, motorSpeeds[0]);
  analogWrite(enR, motorSpeeds[1]);
}
