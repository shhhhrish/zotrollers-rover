// Include libraries needed for this project
#include <Pixy2.h>  // Allows communication with the Pixy2 cameras
#include <Servo.h>  // Controls the servo motor for the claw
#include <CytronMotorDriver.h>  // Controls the left and right drive motors

// Configure motor drivers with meaningful names
CytronMD leftMotor(PWM_DIR, 3, 4);   // Left motor: PWM on Pin 3, DIR on Pin 4
CytronMD rightMotor(PWM_DIR, 9, 10); // Right motor: PWM on Pin 9, DIR on Pin 10

// Create objects for Pixy2 camera and claw servo
Pixy2 pixy;
Servo claw;

// Constants
const int SENSOR1 = A0;
const int SENSOR2 = A1;
const int SENSOR3 = A2;
const int CENTER_X = 150;     // The x-position that represents the center of the camera's view
const int TARGET_WIDTH = 200; // The object's width when it is close enough to grab
const float Kp = 1.5;         // Proportional gain for turning control (used for feedback)
const int MAX_SPEED = 255;    // Maximum motor speed
const int MIN_SPEED = 50;     // Minimum motor speed (ensures the robot moves instead of stalling)
const int FORWARD_SPEED = 150; // Speed for moving forward when approaching the object
const int TURN_THRESHOLD = 5; // How close the object must be to the center before moving forward

void setup() {
    Serial.begin(9600);
    pinMode(SENSOR1, INPUT);
    pinMode(SENSOR2, INPUT);
    pinMode(SENSOR3, INPUT);
    claw.attach(5);  // Attach the servo motor to digital pin 5
    claw.write(0);   // Start with the claw open
    pixy.init();     // Initialize Pixy2 camera
}

void loop() {
    /*
    // Read Sensors
    int s1 = digitalRead(SENSOR1);
    int s2 = digitalRead(SENSOR2);
    int s3 = digitalRead(SENSOR3);

    s1 *= 1;
    s2 *= 2;
    s3 *= 4;

    Serial.println(s1);
    Serial.println(s2);
    Serial.println(s3);

    switch(s1 + s2 + s3) {
        case 1:
            Serial.println("Left IR");
            break;
        case 2:
            Serial.println("Middle IR");
            break;
        case 3:
            Serial.println("Left + Middle IR");
            break;
        case 4:
            Serial.println("Right IR");
            break;
        case 5:
            Serial.println("Left + Right IR (Wtf)");
            break;
        case 6:
            Serial.println("Middle + Right IR");
            break;
        case 7:
            Serial.println("ALL IR");
            break;
    }
    */
    trackObject(); // Call the function to detect and approach the object
}

// Function to track and approach an object using Pixy2
void trackObject() {
    // Request blocks from pixy
    pixy.ccc.getBlocks();

    if (pixy.ccc.numBlocks > 0) {
        // Get largest block x and witdth
        int object_x = pixy.ccc.blocks[0].m_x;
        int object_width = pixy.ccc.blocks[0].m_width;

        // Calcuate error and turn speed
        int error = object_x - CENTER_X;
        int turn_speed = constrain(abs(error) * Kp, MIN_SPEED, MAX_SPEED);

        // Object is to the right
        if (error > TURN_THRESHOLD) { 
            turnRight(turn_speed);
        }
        // Object is to the left 
        else if (error < -TURN_THRESHOLD) {
            turnLeft(turn_speed);
        }
        else {
            // Check if grabable
            if (object_width >= TARGET_WIDTH) {
                grabObject();
            }
            // Object is centered
            else {
                moveForward(FORWARD_SPEED); // Approach object [cite: 9, 16]
            }
        }
    }
    // No Object detected
    else {
        stopMotors();
    }
}

// Function to stop the robot and close the claw
void grabObject() {
    stopMotors();   // stop motors
    delay(500);     // delat for stability
    claw.write(90); // close claw
}

// Function to move forward
void moveForward(int speed) {
    leftMotor.setSpeed(speed);      // Assuming positive speed is clockwise rotation
    rightMotor.setSpeed(-speed);
}

// Function to turn left
void turnLeft(int speed) {
    leftMotor.setSpeed(-speed);
    rightMotor.setSpeed(-speed);
}

// Function to turn right
void turnRight(int speed) {
    leftMotor.setSpeed(speed);
    rightMotor.setSpeed(speed);
}

// Function to stop motors
void stopMotors() {
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);
}

