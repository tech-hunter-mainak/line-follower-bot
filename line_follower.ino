
const int in1 = 2;
const int in2 = 3;
const int in3 = 4;
const int in4 = 5;

const int enable1 = 9;
const int enable2 = 10;

const int leftSensor = 8;   // Left IR sensor
const int rightSensor = 6;  // Right IR sensor
const int centerSensor = 7; // Center IR sensor

// PID variables
float kp = 18.0;
float ki = 0.0;
float kd = 6.0;
float previous_error = 0;
float integral = 0;
float error = 0;
int base_speed = 150;  // Base speed for the motors

// Function prototypes
void forward(int leftSpeed, int rightSpeed);
void left(int leftSpeed, int rightSpeed);
void right(int leftSpeed, int rightSpeed);
void stop();
void rotate(int leftSpeed, int rightSpeed);

void setup() {
  pinMode(leftSensor, INPUT);
  pinMode(centerSensor, INPUT);
  pinMode(rightSensor, INPUT);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enable1, OUTPUT);
  pinMode(enable2, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  // Read sensor values
  int leftValue = digitalRead(leftSensor);
  int rightValue = digitalRead(rightSensor);
  int centerValue = digitalRead(centerSensor);

  // Print sensor values for debugging
  Serial.print(leftValue);
  Serial.print("\t");
  Serial.print(centerValue);
  Serial.print("\t");
  Serial.print(rightValue);
  Serial.print("\n");

  // Calculate the error based on sensor readings
  error = calculateError(leftValue, centerValue, rightValue);

  // PID calculations
  integral += error;
  float derivative = error - previous_error;
  float correction = (kp * error) + (ki * integral) + (kd * derivative);
  previous_error = error;

  // Adjust motor speeds based on the correction
  int leftMotorSpeed = base_speed + correction;
  int rightMotorSpeed = base_speed - correction;

  // Constrain motor speeds to be between 0 and 255
  leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

  // Move the robot based on the sensor readings and calculated speeds
  if (centerValue == 1 && leftValue == 0 && rightValue == 0) {
    forward(leftMotorSpeed, rightMotorSpeed);  // Move forward
  } 
  else if (centerValue == 1 && leftValue == 1 && rightValue == 0) {
    left(leftMotorSpeed, rightMotorSpeed);     // Slight left turn
  } 
  else if (centerValue == 1 && leftValue == 0 && rightValue == 1) {
    right(leftMotorSpeed, rightMotorSpeed);    // Slight right turn
  } 
  else if (centerValue == 1 && leftValue == 1 && rightValue == 1) {
    forward(leftMotorSpeed, rightMotorSpeed);  // Move straight
  } 
  else if (centerValue == 0 && leftValue == 1 && rightValue == 0) {
    left(leftMotorSpeed, rightMotorSpeed);     // Hard left turn
  } 
  else if (centerValue == 0 && leftValue == 0 && rightValue == 1) {
    // Potential right turn
    forward(leftMotorSpeed, rightMotorSpeed);  // Move forward to check
    delay(100);  // Move forward a bit to check straight path

    int newCenterValue = digitalRead(centerSensor);  // Re-check center sensor
    if (newCenterValue == 0) { // No straight path detected
      rotateRightUntilAligned();  // Confirmed right turn
    } else {
      right(leftMotorSpeed, rightMotorSpeed);  // Follow the slight right turn
    }
  } 
  else if (centerValue == 0 && leftValue == 1 && rightValue == 1) {
    left(leftMotorSpeed / 2, rightMotorSpeed); // Sharper left
  } 
  else if (centerValue == 0 && leftValue == 0 && rightValue == 0) {
    stop(); // Stop and perform a rotation to search for line
    rotate(70, 70);
  }

  delay(5);  // Small delay for smoother operation
}

// Function to calculate error based on sensor readings
float calculateError(int leftValue, int centerValue, int rightValue) {
  if (leftValue == 1) return -1;  // Left sensor on line
  if (centerValue == 1) return 0; // Center sensor on line
  if (rightValue == 1) return 1;  // Right sensor on line
  return previous_error;           // Maintain last known error if no line detected
}

// Function to move forward
void forward(int leftSpeed, int rightSpeed) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enable1, leftSpeed);
  analogWrite(enable2, rightSpeed);
}

// Function to turn left
void left(int leftSpeed, int rightSpeed) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enable1, leftSpeed);
  analogWrite(enable2, rightSpeed);
}

// Function to turn right
void right(int leftSpeed, int rightSpeed) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enable1, leftSpeed);
  analogWrite(enable2, rightSpeed);
}

// Function to stop the robot
void stop() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enable1, 0);
  analogWrite(enable2, 0);
}

// Function to rotate the robot to search for the line
void rotate(int leftSpeed, int rightSpeed) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enable1, leftSpeed);
  analogWrite(enable2, rightSpeed);
}

// Custom function to rotate right for junction
void rotateRightUntilAligned() {
  // Rotate right until the center sensor detects the line again
  while (digitalRead(centerSensor) == 0) {
    right(100, 100);  // Adjust speed for precise rotation
    delay(10);      // Small delay to ensure smooth rotation
  }
  stop();           // Stop after re-alignment
}
