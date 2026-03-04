// Include libraries
//#include <Pixy2.h>            // Allows communication with the Pixy2 cameras
//#include <Servo.h>            // Controls the servo motor for the claw
#include <CytronMotorDriver.h>  // Controls the left and right drive motors
#include <PID_v1.h>             // PID library

// Configure motor drivers
CytronMD leftMotor(PWM_DIR, 11, 13);    // Left motor: PWM on Pin 11, DIR on Pin 13
CytronMD rightMotor(PWM_DIR, 9, 8);     // Right motor: PWM on Pin 9, DIR on Pin 8

/*
// Create objects for Pixy2 camera and claw servo
Pixy2 pixy;
Servo claw;
*/

// IR Sensor Pins
const int SENSOR1 = A0;     // Left
const int SENSOR2 = A1;     // Center
const int SENSOR3 = A2;     // Right

// Line-following sensor weights
//  Left = -1, Center = 0, Right = +1
//  Sensor reading * weight is sent to PID
const double SENSOR1_WEIGHT = -1.0;  // Left
const double SENSOR2_WEIGHT =  0.0;  // Center
const double SENSOR3_WEIGHT =  1.0;  // Right

// Motor speed constants
const int BASE_SPEED   = 150;   // Default speed
const int MAX_SPEED    = 255;   // Max speed
const int MIN_SPEED    = 60;    // Min speed (not 0 to prevent stall)

// PID constants
double Kp = 120.0;  // proportional to current error
double Ki =   0.5;  // accumulated error (removes steady-state)
double Kd =  15.0;  // rate of change  (dampens oscillation)

// PID working variables (updated every loop)
double pidInput    = 0.0;   // Current error  (fed IN)
double pidOutput   = 0.0;   // Correction     (fed OUT)
double pidSetpoint = 0.0;   // Desired error  (always 0 = on the line)

// Construct the PID object
PID linePID(&pidInput, &pidOutput, &pidSetpoint, Kp, Ki, Kd, DIRECT);

// Lost-line state
enum RecoveryDir { NONE, RECOVER_LEFT, RECOVER_RIGHT };
RecoveryDir lastTurnDir = NONE;

/*
// Pixy Constants
const int CENTER_X = 150;
const int TARGET_WIDTH = 200;
const float Kp_vision = 1.5;
const int TURN_THRESHOLD = 5;
*/

void setup() {
    Serial.begin(9600);
    pinMode(SENSOR1, INPUT);
    pinMode(SENSOR2, INPUT);
    pinMode(SENSOR3, INPUT);

    // Limit PID output to the maximum speed range
    linePID.SetOutputLimits(-MAX_SPEED, MAX_SPEED);

    // Set PID sample time (10 ms = 100 Hz)
    linePID.SetSampleTime(10);

    // Enable continuous mode
    linePID.SetMode(AUTOMATIC);

    /*
    claw.attach(5);
    claw.write(0);
    pixy.init();
    */
}

void loop() {
    // Read sensors (1 = on line, 0 = off line)
    int s1 = digitalRead(SENSOR1);
    int s2 = digitalRead(SENSOR2);
    int s3 = digitalRead(SENSOR3);

    int activeSensors = s1 + s2 + s3;

    // Calculate weighted-average error
    //  error > 0  line is to the right
    //  error < 0  line is to the left
    //  error = 0  line is centered
    if (activeSensors > 0) {
        pidInput = (s1 * SENSOR1_WEIGHT +
                    s2 * SENSOR2_WEIGHT +
                    s3 * SENSOR3_WEIGHT) / activeSensors;

        // Remember which side the line was last seen on for recovery function
        if (pidInput < 0) {
            lastTurnDir = RECOVER_RIGHT;    // line was left -> search right
        }
        if (pidInput > 0) {
            lastTurnDir = RECOVER_LEFT;     // line was right -> search left
        }
    }

    // Line recovery
    if (activeSensors == 0) {
        recoverLine();
        return;     // Skip normal PID loop until line found
    }

    // Run PID
    linePID.Compute();

    // Take pid output and turn into motor instructions
    //  + pidOutput -> line is right -> speed up right, slow left
    //  - pidOutput -> line is left  -> speed up left, slow right
    int leftSpeed  = constrain((int)(BASE_SPEED - pidOutput), -MAX_SPEED, MAX_SPEED);
    int rightSpeed = constrain((int)(BASE_SPEED + pidOutput), -MAX_SPEED, MAX_SPEED);

    // Enforce minimum speed in the forward direction
    if (leftSpeed  > 0) {
        leftSpeed  = max(leftSpeed,  MIN_SPEED);
    }
    if (rightSpeed > 0) {
        rightSpeed = max(rightSpeed, MIN_SPEED);
    }

    driveMotors(leftSpeed, rightSpeed);

// Debug helpers
    Serial.print("S1:");  Serial.print(s1);
    Serial.print(" S2:"); Serial.print(s2);
    Serial.print(" S3:"); Serial.print(s3);
    Serial.print(" Err:"); Serial.print(pidInput,  3);      // limit output to 3 decimals
    Serial.print(" PID:");  Serial.print(pidOutput,  1);    // limit output to 1 decimals
    Serial.print(" L:");    Serial.print(leftSpeed);
    Serial.print(" R:");    Serial.println(rightSpeed);
}

// Spin in last known direction to search for the line
void recoverLine() {
    Serial.println("LINE LOST - recovering");
    if (lastTurnDir == RECOVER_LEFT) {
        driveMotors(-MIN_SPEED, MIN_SPEED);   // spin left
    } 
    else if (lastTurnDir == RECOVER_RIGHT) {
        driveMotors(MIN_SPEED, -MIN_SPEED);   // spin right
    } 
    else {
        stopMotors();   // no last known, stop
    }
}

// Motor Helpers

// Accounts for mechanical setup (positive = forward)
void driveMotors(int leftSpeed, int rightSpeed) {
    leftMotor.setSpeed(leftSpeed);
    rightMotor.setSpeed(-rightSpeed);  // Right motor is mechanically reversed
}

// Move forward
void moveForward(int speed) {
    driveMotors(speed, speed);
}

// Turn left
void turnLeft(int speed) {
    driveMotors(-speed, speed);
}

// Turn right
void turnRight(int speed) {
    driveMotors(speed, -speed);
}

// Stop
void stopMotors() {
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);
}

/*
// ──────────────────────────────────────────────
// Pixy2 object tracking (kept for future use)
// ──────────────────────────────────────────────
const int CENTER_X     = 150;
const int TARGET_WIDTH = 200;
const float Kp_vision  = 1.5;
const int MAX_SPEED_V  = 255;
const int MIN_SPEED_V  = 50;
const int FORWARD_SPEED = 150;
const int TURN_THRESHOLD = 5;

void trackObject() {
    pixy.ccc.getBlocks();
    if (pixy.ccc.numBlocks > 0) {
        int object_x     = pixy.ccc.blocks[0].m_x;
        int object_width = pixy.ccc.blocks[0].m_width;
        int error        = object_x - CENTER_X;
        int turn_speed   = constrain(abs(error) * Kp_vision, MIN_SPEED_V, MAX_SPEED_V);

        if (error > TURN_THRESHOLD)       turnRight(turn_speed);
        else if (error < -TURN_THRESHOLD) turnLeft(turn_speed);
        else {
            if (object_width >= TARGET_WIDTH) grabObject();
            else                              moveForward(FORWARD_SPEED);
        }
    } else {
        stopMotors();
    }
}

void grabObject() {
    stopMotors();
    delay(500);
    claw.write(90);
}
*/
