#include <Servo.h>

// Pin Definitions
#define IR1 A0 // Left-most IR Sensor
#define IR2 A1 // Middle-left IR Sensor
#define IR3 A2 // Middle-right IR Sensor
#define IR4 A3 // Right-most IR Sensor
#define TRIG 9 // Ultrasonic sensor trig pin
#define ECHO 10 // Ultrasonic sensor echo pin
#define LEFT_MOTOR_FORWARD 3
#define LEFT_MOTOR_BACKWARD 5
#define RIGHT_MOTOR_FORWARD 6
#define RIGHT_MOTOR_BACKWARD 11

Servo gripperServo;

// Thresholds
#define IR_THRESHOLD 500
#define DISTANCE_THRESHOLD 10 // in cm

// Robot States
bool objectDetected = false;

void setup() {
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);

  gripperServo.attach(7);
  gripperServo.write(90); // Gripper in open position
  
  Serial.begin(9600);
}

void loop() {
  int ir1 = analogRead(IR1);
  int ir2 = analogRead(IR2);
  int ir3 = analogRead(IR3);
  int ir4 = analogRead(IR4);

  int distance = measureDistance();

  if (distance < DISTANCE_THRESHOLD) {
    objectDetected = true;
    stopMotors();
    pickUpObject();
    moveToBox();
    placeObject();
    objectDetected = false;
  } else {
    followLine(ir1, ir2, ir3, ir4);
  }
}

void followLine(int ir1, int ir2, int ir3, int ir4) {
  // Logic to follow line based on IR sensor readings
  if (ir2 < IR_THRESHOLD && ir3 < IR_THRESHOLD) { // Move forward
    moveForward();
  } else if (ir1 < IR_THRESHOLD) { // Slight left
    turnLeft();
  } else if (ir4 < IR_THRESHOLD) { // Slight right
    turnRight();
  } else { // Stop in case of unexpected scenario
    stopMotors();
  }
}

void pickUpObject() {
  gripperServo.write(90); // Open gripper
  delay(500);
  gripperServo.write(0); // Close gripper
  delay(500);
}

void moveToBox() {
  // Move forward for a set duration to reach the black box
  moveForward();
  delay(2000); // Adjust delay based on track layout
  stopMotors();
}

void placeObject() {
  gripperServo.write(90); // Open gripper to place object
  delay(500);
}

int measureDistance() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  
  long duration = pulseIn(ECHO, HIGH);
  int distance = duration * 0.034 / 2; // Convert to cm
  return distance;
}

void moveForward() {
  analogWrite(LEFT_MOTOR_FORWARD, 255);
  analogWrite(RIGHT_MOTOR_FORWARD, 255);
  analogWrite(LEFT_MOTOR_BACKWARD, 0);
  analogWrite(RIGHT_MOTOR_BACKWARD, 0);
}

void turnLeft() {
  analogWrite(LEFT_MOTOR_FORWARD, 0);
  analogWrite(RIGHT_MOTOR_FORWARD, 255);
  analogWrite(LEFT_MOTOR_BACKWARD, 0);
  analogWrite(RIGHT_MOTOR_BACKWARD, 0);
}

void turnRight() {
  analogWrite(LEFT_MOTOR_FORWARD, 255);
  analogWrite(RIGHT_MOTOR_FORWARD, 0);
  analogWrite(LEFT_MOTOR_BACKWARD, 0);
  analogWrite(RIGHT_MOTOR_BACKWARD, 0);
}

void stopMotors() {
  analogWrite(LEFT_MOTOR_FORWARD, 0);
  analogWrite(RIGHT_MOTOR_FORWARD, 0);
  analogWrite(LEFT_MOTOR_BACKWARD, 0);
  analogWrite(RIGHT_MOTOR_BACKWARD, 0);
}