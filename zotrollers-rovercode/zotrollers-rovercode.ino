//  Zotrollers Rover
//  Possible States
//    LINE_FOLLOW  -> (stop line detected)
//    STOP_CONFIRM -> (held long enough)
//    APPROACH_CAN -> (Pixy tracks can until close)
//    GRAB_CAN     -> (servo closes, done)

// ------- Libraries -------
#include <CytronMotorDriver.h>  // Controls the left and right drive motors
#include <PID_v1.h>             // PID library
#include <Pixy2.h>              // Allows communication with the Pixy2 cameras
//#include <Servo.h>              // Controls the servo motor for the claw

// ------- Hardware Configuration -------
CytronMD leftMotor(PWM_DIR, 6, 7);    // Left motor: PWM on Pin 11, DIR on Pin 13
CytronMD rightMotor(PWM_DIR, 3, 2);     // Right motor: PWM on Pin 9, DIR on Pin 8

Pixy2 pixy;
//Servo claw;

const int SENSOR1 = A0;     // Left
const int SENSOR2 = A1;     // Center
const int SENSOR3 = A2;     // Right

// ------- Sensor Constants -------
// Line-following sensor weights
//  Left = -1, Center = 0, Right = +1
//  Sensor reading * weight is sent to PID
const double SENSOR1_WEIGHT = -1.0;  // Left
const double SENSOR2_WEIGHT =  0.0;  // Center
const double SENSOR3_WEIGHT =  1.0;  // Right

// ------- Motor Constants -------
const int BASE_SPEED = 150;   // Default speed
const int MAX_SPEED = 255;   // Max speed
const int MIN_SPEED = 20;    // Min speed
const int RECOVER_SPEED = 60;   // Speed when recovering

// ------- PID Constants -------
double Kp = 95.0;   // proportional to current error
double Ki = 0.4;    // accumulated error (removes steady-state)
double Kd = 0.8;    // rate of change  (dampens oscillation)

// ------- PID Variables -------
double pidInput = 0.0;   // Current error  (fed IN)
double pidOutput = 0.0;   // Correction     (fed OUT)
double pidSetpoint = 0.0;   // Desired error  (always 0 = on the line)

// Construct the PID object
PID linePID(&pidInput, &pidOutput, &pidSetpoint, Kp, Ki, Kd, DIRECT);

// ------- Sensor Sampling Constants -------
const int SAMPLE_COUNT = 5;   // readings per sensor per loop
const int SAMPLE_THRESHOLD = 3;   // minimum high reads to count as ON

// ------- Recovery Constants -------
const unsigned long LOSS_MS = 1200;
unsigned long lineLostAt = 0;
bool linePreviouslyLost = false;

// Lost-line state
enum RecoveryDir { NONE, RECOVER_LEFT, RECOVER_RIGHT };
RecoveryDir lastTurnDir = NONE;

// ------- Stop Line Constants -------
const unsigned long STOP_CONFIRM_MS = 500;
unsigned long stopLineSeenAt = 0;
bool stopLineArmed = false;

// ------- Pixy Constants -------
const int PIXY_CENTER_X = 158;
const int PIXY_TURN_THRESH =  10;
const int PIXY_GRAB_WIDTH = 180;
const int APPROACH_SPEED = 150;
const int PIXY_TURN_SPEED =  90;
const float Kp_vision =  1.2;

// ------- Servo Constants -------
const int CLAW_OPEN = 0;
const int CLAW_CLOSED = 90;

// Rover State Tracker
enum RoverState {
    LINE_FOLLOW,    // PID line following
    STOP_CONFIRM,   // All sensors triggered – waiting to confirm stop line
    APPROACH_CAN,   // Pixy-guided approach
    GRAB_CAN        // Servo close – terminal state
};
RoverState state = LINE_FOLLOW;

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

    //claw.attach(9);
    //claw.write(CLAW_OPEN);
    pixy.init();

    Serial.println("Rover: Line Follow");
}

void loop() {
    switch (state) {
        case LINE_FOLLOW:
            runLineFollow();
            break;

        case STOP_CONFIRM:
            runStopConfirm();
            break;

        case APPROACH_CAN:
            runApproachCan();
            break;

        case GRAB_CAN:
            runGrabCan();
            break;
    }
}

// ------ Read Sensors ------ 
// helper function to read and take average of sensor input
int readSensor(int pin) {
    int count = 0;
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        count += digitalRead(pin);
    }
    if (count >= SAMPLE_THRESHOLD) {
        return 1;
    }
    else {
        return 0;
    }
}

// ------ PID line following ------
void runLineFollow() {
    // Each sensor is sampled 5 times
    int s1 = readSensor(SENSOR1);
    int s2 = readSensor(SENSOR2);
    int s3 = readSensor(SENSOR3);
    int activeSensors = s1 + s2 + s3;

    // All three sensors on -> potential stop line
    if (activeSensors == 3) {
        if (!stopLineArmed) {
            stopLineArmed  = true;
            stopLineSeenAt = millis();
            Serial.println("Potential Stop Line");
        }
        driveMotors(BASE_SPEED / 2, BASE_SPEED / 2);

        if (millis() - stopLineSeenAt >= STOP_CONFIRM_MS) {
            stopMotors();
            state = STOP_CONFIRM;
            Serial.println("------ STOP LINE CONFIRMED ------");
        }
        return;
    }

    // Cancel stop line if sensors drop
    stopLineArmed = false;

    // Update PID input
    if (activeSensors > 0) {
        linePreviouslyLost = false;
        lineLostAt = 0;

        pidInput = (s1 * SENSOR1_WEIGHT +
                    s2 * SENSOR2_WEIGHT +
                    s3 * SENSOR3_WEIGHT) / activeSensors;

        if (pidInput < 0) lastTurnDir = RECOVER_RIGHT;
        if (pidInput > 0) lastTurnDir = RECOVER_LEFT;

        linePID.Compute();

        int leftSpeed  = constrain((int)(BASE_SPEED - pidOutput), -MAX_SPEED, MAX_SPEED);
        int rightSpeed = constrain((int)(BASE_SPEED + pidOutput), -MAX_SPEED, MAX_SPEED);

        if (leftSpeed  > 0) leftSpeed  = max(leftSpeed,  MIN_SPEED);
        if (rightSpeed > 0) rightSpeed = max(rightSpeed, MIN_SPEED);

        driveMotors(leftSpeed, rightSpeed);

        Serial.print("S1:"); Serial.print(s1);
        Serial.print(" S2:"); Serial.print(s2);
        Serial.print(" S3:"); Serial.print(s3);
        Serial.print(" Err:"); Serial.print(pidInput, 3);
        Serial.print(" PID:"); Serial.print(pidOutput, 1);
        Serial.print(" L:"); Serial.print(leftSpeed);
        Serial.print(" R:"); Serial.println(rightSpeed);
    } 
    // Line not visible: wait before entering recovery
    else {
        if (!linePreviouslyLost) {
            // First tick with no line – start the loss timer
            linePreviouslyLost = true;
            lineLostAt = millis();
        }

        if (millis() - lineLostAt < LOSS_MS) {
            // Gap is short: dont react yet
            int leftSpeed  = constrain((int)(BASE_SPEED - pidOutput), -MAX_SPEED, MAX_SPEED);
            int rightSpeed = constrain((int)(BASE_SPEED + pidOutput), -MAX_SPEED, MAX_SPEED);
            if (leftSpeed  > 0) leftSpeed  = max(leftSpeed,  MIN_SPEED);
            if (rightSpeed > 0) rightSpeed = max(rightSpeed, MIN_SPEED);
            driveMotors(leftSpeed, rightSpeed);
            Serial.println("Sensor gap: holding last output");
        } 
        else {
            // Line is genuinely lost, enter recovery
            recoverLine();
        }
    }
}

// ------ Stop Confirm ------
//  Robot is stopped at the line, wait then hand off to pixy
void runStopConfirm() {
    stopMotors();
    delay(300);
    state = APPROACH_CAN;
    Serial.println("=== → APPROACH_CAN ===");
}

// ------ Approch Can ------
// Use Pixy2 to locate the can, steer toward it, and advance until close enough to grab
void runApproachCan() {
    pixy.ccc.getBlocks();

    if (pixy.ccc.numBlocks == 0) {
        // Can not visible, spin slowly to search
        Serial.println("Can not foundw, searching...");
        driveMotors(-PIXY_TURN_SPEED, PIXY_TURN_SPEED);   // spin left to search
        return;
    }

    // Use the largest block (index 0 – Pixy sorts by area)
    int obj_x = pixy.ccc.blocks[0].m_x;
    int obj_width = pixy.ccc.blocks[0].m_width;

    int error = obj_x - PIXY_CENTER_X;
    int turn_speed = constrain((int)(abs(error) * Kp_vision),
                                0, MAX_SPEED - APPROACH_SPEED);

    Serial.print("Pixy x:"); 
    Serial.print(obj_x);
    Serial.print(" w:");     
    Serial.print(obj_width);
    Serial.print(" err:");   
    Serial.println(error);

    // Check if grabable
    if (obj_width >= PIXY_GRAB_WIDTH) {
        stopMotors();
        state = GRAB_CAN;
        Serial.println("------ GRAB_CAN ------");
        return;
    }

    // Steer toward the can while moving forward
    if (error > PIXY_TURN_THRESH) {
        // Can is to the right -> turn right
        driveMotors(APPROACH_SPEED + turn_speed,
                    APPROACH_SPEED - turn_speed);
    } 
    else if (error < -PIXY_TURN_THRESH) {
        // Can is to the left -> turn left
        driveMotors(APPROACH_SPEED - turn_speed,
                    APPROACH_SPEED + turn_speed);
    } 
    else {
        // Centred –> drive straight
        driveMotors(APPROACH_SPEED, APPROACH_SPEED);
    }
}

// ------ Grab Can ------
// Stop, close claw, stay stopped
void runGrabCan() {
    stopMotors();
    delay(250);
    //claw.write(CLAW_CLOSED);
    Serial.println("------ CAN GRABBED ------");

    // Hold forever
    while (true) { 
        delay(1000); 
    }
}

// ------ Line-lost Recovery ------
void recoverLine() {
    Serial.println("LINE LOST -> recovering");
    if (lastTurnDir == RECOVER_LEFT) {
        driveMotors(-RECOVER_SPEED,  RECOVER_SPEED);
    } 
    else if (lastTurnDir == RECOVER_RIGHT) {
        driveMotors( RECOVER_SPEED, -RECOVER_SPEED);
    } 
    else {
        stopMotors();
    }
}

// Motor Helpers

void driveMotors(int leftSpeed, int rightSpeed) {
    leftMotor.setSpeed(-leftSpeed);    // left motor mechanically reversed
    rightMotor.setSpeed(rightSpeed);
}
void stopMotors() {
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);
}