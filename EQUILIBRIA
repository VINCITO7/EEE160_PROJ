#include <Servo.h>

// Pin definitions
const int TRIG_PIN = 9;
const int ECHO_PIN = 10;
const int SERVO_PIN = 11;

// Servo object
Servo servo;

// Distance measurements
double distance;
const double setPoint = 4.5;  // Desired distance in inches

// PID Constants (tuned for better performance)
double Kp = 1;    // Proportional gain
double Ki = 0.5;    // Integral gain
double Kd = 0.9 ;    // Derivative gain

// PID Variables
double previousError = 0;
double totalError = 0;
const double maxIntegral = 30.0; // Limit to prevent integral wind-up

// Timing variables
unsigned long previousTime = 0;
const unsigned long controlInterval = 50; // Control loop every 30 ms
  
// Moving average filter variables
const int numReadings = 5;
double readings[numReadings];
int readIndex = 0;
double total = 0;
double averageDistance = 0;

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  servo.attach(SERVO_PIN);
  servo.write(200); // Initial servo position (neutral)
  Serial.begin(9600);

  // Initialize readings array with setPoint
  for (int i = 0; i < numReadings; i++) {
    readings[i] = setPoint;
    total += setPoint;
  }
  averageDistance = setPoint;
}

void loop() {
  unsigned long currentTime = millis();

  // Run the control loop at defined intervals
  if (currentTime - previousTime >= controlInterval) {
    previousTime = currentTime;
    controlBeam();
  }
}

void controlBeam() {
  distance = measureDistance();
  updateMovingAverage(distance);

  Serial.print("Distance: ");
  Serial.print(averageDistance);
  Serial.println(" inches");

  // PID calculation
  double error = setPoint - averageDistance;

  // Proportional term
  double proportional = Kp * error;

  // Integral term with wind-up prevention
  totalError += error;
  totalError = constrain(totalError, -maxIntegral, maxIntegral);
  double integral = Ki * totalError;

  // Derivative term
  double derivative = Kd * (error - previousError);

  // Calculate correction
  double correction = proportional + integral + derivative;

  previousError = error;

  // Adjust servo position
  int servoPosition = 75 + (int)correction;
  servoPosition = constrain(servoPosition, 0, 180);
  servo.write(servoPosition);

  Serial.print("Servo Position: ");
  Serial.println(servoPosition);
}

// Function to measure distance using HC-SR04
double measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(1 );
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // Timeout at 30 ms (to avoid hanging)
  if (duration == 0) {
    Serial.println("Error: No echo received");
    return setPoint; // Return setPoint on timeout for stability
  }

  double dist = duration * 0.0133; // Convert to inches (duration * 0.034 / 2 * 0.3937)
  return dist;
}

// Moving average filter to smooth out measurements
void updateMovingAverage(double newReading) {
  total -= readings[readIndex];
  readings[readIndex] = newReading;
  total += newReading;
  readIndex = (readIndex + 1) % numReadings;
  averageDistance = total / numReadings;
}
