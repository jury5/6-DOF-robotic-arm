# 6-DOF-robotic-arm

#include <Servo.h> // Include the Servo library
#include <math.h>  // For trigonometric functions

// Create servo objects for each joint
Servo shoulderServo;
Servo elbowServo;
Servo wristPitchServo;
Servo wristRollServo;
Servo gripperServo;

// Define servo pin connections
#define SHOULDER_PIN 5
#define ELBOW_PIN 6
#define WRIST_PITCH_PIN 9
#define WRIST_ROLL_PIN 10
#define GRIPPER_PIN 11

// Robotic arm dimensions (in cm, adjust these based on your arm design)
const float L1 = 10.0; // Length of the shoulder link
const float L2 = 10.0; // Length of the elbow link
const float L3 = 5.0;  // Length of the wrist link

void setup() {
  // Attach the servos to their respective pins
  shoulderServo.attach(SHOULDER_PIN);
  elbowServo.attach(ELBOW_PIN);
  wristPitchServo.attach(WRIST_PITCH_PIN);
  wristRollServo.attach(WRIST_ROLL_PIN);
  gripperServo.attach(GRIPPER_PIN);

  // Move servos to initial positions
  moveToInitialPosition();

  Serial.begin(9600); // Start serial communication for debugging
  Serial.println("Robotic Arm Initialized");
}

void loop() {
  // Example coordinates for pickup and placement (in cm)
  float pickupX = 10.0, pickupY = 10.0, pickupZ = 5.0;
  float placeX = 5.0, placeY = 15.0, placeZ = 5.0;

  // Perform the pick-and-place operation
  pickAndPlace(pickupX, pickupY, pickupZ, placeX, placeY, placeZ);
  delay(5000); // Wait before repeating the process
}

// Function to move to initial position
void moveToInitialPosition() {
  shoulderServo.write(90);  // Neutral position
  elbowServo.write(90);     // Neutral position
  wristPitchServo.write(90);// Neutral position
  wristRollServo.write(90); // Neutral position
  gripperServo.write(0);    // Gripper open
  delay(1000);
}

// Function to perform pick-and-place operation
void pickAndPlace(float pickupX, float pickupY, float pickupZ, float placeX, float placeY, float placeZ) {
  // Move to pickup position
  moveToPosition(pickupX, pickupY, pickupZ);
  closeGripper(); // Close gripper to grab the object
  delay(1000);

  // Move to placement position
  moveToPosition(placeX, placeY, placeZ);
  openGripper(); // Open gripper to release the object
  delay(1000);

  // Return to initial position
  moveToInitialPosition();
}

// Function to move the arm to a specific position (x, y, z)
void moveToPosition(float x, float y, float z) {
  // Calculate shoulder and elbow angles using inverse kinematics
  float r = sqrt(x * x + y * y); // Horizontal distance
  float s = z - L3;             // Vertical height minus wrist link
  float d = sqrt(r * r + s * s); // Distance to target point

  // Check if the position is reachable
  if (d > (L1 + L2) || d < fabs(L1 - L2)) {
    Serial.println("Target position out of reach!");
    return;
  }

  // Inverse kinematics calculations
  float angleA = atan2(s, r); // Shoulder angle relative to horizontal
  float angleB = acos((L1 * L1 + d * d - L2 * L2) / (2 * L1 * d)); // Law of cosines for shoulder
  float shoulderAngle = degrees(angleA + angleB); // Combine angles for shoulder servo

  float angleC = acos((L1 * L1 + L2 * L2 - d * d) / (2 * L1 * L2)); // Law of cosines for elbow
  float elbowAngle = degrees(angleC); // Elbow servo angle

  // Wrist pitch adjustment for vertical alignment
  float wristPitchAngle = 180 - (shoulderAngle + elbowAngle);

  // Set servo positions
  shoulderServo.write(shoulderAngle);
  elbowServo.write(180 - elbowAngle); // Reverse for elbow servo orientation
  wristPitchServo.write(wristPitchAngle);

  Serial.print("Moving to (");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.print(z);
  Serial.println(")");

  delay(1000); // Allow time for servos to reach the position
}

// Function to close the gripper
void closeGripper() {
  gripperServo.write(45); // Adjust based on your gripper's range
  Serial.println("Gripper closed");
  delay(500);
}

// Function to open the gripper
void openGripper() {
  gripperServo.write(0); // Adjust based on your gripper's range
  Serial.println("Gripper opened");
  delay(500);
}
