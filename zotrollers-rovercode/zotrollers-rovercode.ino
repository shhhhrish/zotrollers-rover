/* -------------------------------------------------------
  Team:     Zotrollers
  Members:  Shrish Potla, Vince Lu, Mateyo Leon
            Neeraj Bhuvaneshwaran, Sean Amet
  Last Edit:     3/12/2026

    Project:  Winter 2026 ENGR 7B – Autonomous Rover
    Summary:  Autonomous rover that follows a black tape line
            using three IR sensors and a PID controller.
            On detecting a horizontal stop line, the rover
            halts and switches to vision navigation,
            using a Pixy to locate and approach a red coloured
            can before closing a two-servo claw to grab it and 
            subsequently lift it

  Possible States:
    LINE_FOLLOW  -> stop line detected
    STOP_CONFIRM -> held long enough
    APPROACH_CAN -> Pixy tracks can until close
    GRAB_CAN     -> servo closes claw and lifts, done
 ---------------------------------------------------------*/

// ------- Libraries -------
#include <CytronMotorDriver.h>  // Controls the left and right drive motors
#include <PID_v1.h>             // PID library
#include <Pixy2.h>              // Allows communication with Pixy camera
#include <Servo.h>              // Controls the servo motors for the claw

// ------- Hardware Configuration -------
CytronMD leftMotor(PWM_DIR, 6, 7);  // Left motor: PWM on Pin 6, DIR on Pin 7
CytronMD rightMotor(PWM_DIR, 3, 2); // Right motor: PWM on Pin 3, DIR on Pin 2

// Two servos control the claw:
//  clawLift  – raises/lowers the arm after grabbing
//  clawClose – opens/closes the claw around the can
Servo clawLift;
Servo clawClose;

// Pixy camera object for colour block detection
Pixy2 pixy;

// ------- IR Sensor Pins -------
const int SENSOR1 = A0; // Left sensor
const int SENSOR2 = A1; // Center sensor
const int SENSOR3 = A2; // Right sensor

// ------- Sensor Constants -------
/* Each sensor is assigned a position weight centred at 0 
    (the robot's centre). The weighted average of active sensors 
    is fed into the PID controller

    Negative = line is to the left
    Zero     = line is centred
    Positive = line is to the right */
const double SENSOR1_WEIGHT = -1.0;  // Left
const double SENSOR2_WEIGHT =  0.0;  // Center
const double SENSOR3_WEIGHT =  1.0;  // Right

// ------- Motor Constants -------
const int BASE_SPEED = 150;   // Default speed (0-255)
const int MAX_SPEED = 255;   // Max speed
const int MIN_SPEED = 20;    // Min speed
const int RECOVER_SPEED = 60;   // Slow speed used when recovering

// ------- PID Constants -------
/* Kp – scales correction to current error (main steering force)
   Ki – corrects accumulated long-term drift
   Kd – dampens oscillation by reacting to rate of change */
double Kp = 95.0;
double Ki = 0.4;
double Kd = 0.8; 

// ------- PID Variables -------
double pidInput = 0.0;      // Weighted sensor error fed into the PID
double pidOutput = 0.0;     // Steering correction output by the PID
double pidSetpoint = 0.0;   // Target error (always 0: rover should be centred)

// Construct the PID object
PID linePID(&pidInput, &pidOutput, &pidSetpoint, Kp, Ki, Kd, DIRECT);

// ------- Sensor Sampling Constants -------
// Each sensor is read multiple times per tick to filter out noise
// A sensor only counts as ON if enough of those reads are HIGH
const int SAMPLE_COUNT = 5;   // readings per sensor per loop
const int SAMPLE_THRESHOLD = 3;   // minimum high reads to count as ON

// ------- Recovery Constants -------
// Before entering recovery mode, the rover waits
// to confirm the line is genuinely gone
const unsigned long LOSS_MS = 1200; // Time (ms) to hold last output before recovering
unsigned long lineLostAt = 0;       // Timestamp of when the line was first lost in ms
bool linePreviouslyLost = false;    // True once the loss timer has started

// Tracks which direction the line was last seen so recovery
// spins the rover back toward it
enum RecoveryDir { NONE, RECOVER_LEFT, RECOVER_RIGHT };
RecoveryDir lastTurnDir = NONE;

// ------- Stop Line Constants -------
// A stop line is detected when all three sensors are ON simultaneously
// A confirmation timer prevents false triggers
const unsigned long STOP_CONFIRM_MS = 500;  // Time (ms) all-3 must stay ON to confirm
unsigned long stopLineSeenAt = 0;           // Timestamp of first all-3 reading
bool stopLineArmed  = false;                // True once the first all-3 reading occurs

// ------- Pixy Constants -------
const int PIXY_CENTER_X = 158;      // Midpoint of Pixy frame
const int PIXY_TURN_THRESH = 10;    // Error within this are ignored
const int PIXY_GRAB_WIDTH = 180;    // Block width at which can is close enough to grab
const int APPROACH_SPEED = 150;     // Forward speed used when approching
const int PIXY_TURN_SPEED = 90;     // Spin speed used when approching
const float Kp_vision = 1.2;        // Proprtional gain for Pixy steering

// ------- Servo Constants -------
const int CLAW_OPEN = 140;      // Servo angle used for open claw
const int CLAW_CLOSED = 40;     // Servo angle used for closed claw
const int CLAW_DOWN = 0;        // Arm lowered position (ready to grab)
const int CLAW_UP = 40;         // Arm raised position (holding can)

// ------- Rover State Tacker -------
// Rover behaviour is divided into four states
// Each state runs its own function and decides when to transition to next
enum RoverState {
    LINE_FOLLOW,    // Follow tape line using PID
    STOP_CONFIRM,   // Pausing after stop line detection before switching to camera
    APPROACH_CAN,   // Using Pixy to navigate toward the can
    GRAB_CAN        // Closing the claw and lifting the can (final state)
};
RoverState state = LINE_FOLLOW; // Rover starts in line follow mode

// -------------------------------------------------------------
void setup() {
    Serial.begin(9600);

    // Configure IR sensor pins as inputs
    pinMode(SENSOR1, INPUT);
    pinMode(SENSOR2, INPUT);
    pinMode(SENSOR3, INPUT);

    // Limit PID output to the maximum speed range
    linePID.SetOutputLimits(-MAX_SPEED, MAX_SPEED);

    // Set PID sample time (10 ms = 100 Hz)
    linePID.SetSampleTime(10);

    // Enable continuous mode
    linePID.SetMode(AUTOMATIC);

    // Initialise servos
    clawClose.attach(8);
    clawClose.write(CLAW_OPEN);     // Start with claw open
    clawLift.attach(9);
    clawLift.write(CLAW_UP);      // Start with arm up

    pixy.init();

    Serial.println("------ Line Follow ------");
}

void loop() {
    // Use correct function depending on current state
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
// Samples a single IR sensor pin SAMPLE_COUNT times and returns
// 1 (on line) if SAMPLE_THRESHOLD reads were HIGH, or 0 (off line) otherwise
int readSensor(int pin) {
    int count = 0;
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        count += digitalRead(pin);
    }
    if (count >= SAMPLE_THRESHOLD) {
        return 1;   // Enough reads were HIGH – sensor is on the line
    }
    else {
        return 0;   // Too few HIGH reads – sensor is off the line
    }
}

// ------- Line Follow State -------
// Main line-following logic using PID. Reads the three IR sensors,
// computes average error, and steers the motors to keep
// the error at zero (centred on line)
// Also detects the stop line and manages the line loss timer
void runLineFollow() {
    // Each sensor is sampled SAMPLE_COUNT times
    int s1 = readSensor(SENSOR1);
    int s2 = readSensor(SENSOR2);
    int s3 = readSensor(SENSOR3);
    int activeSensors = s1 + s2 + s3;   // Total number of sensors currently on the line

    // ------- Stop Line Detection -------
    // All three sensors ON simultaneously indicates the stop line
    // Required hold for STOP_CONFIRM_MS to avoid false trigger
    if (activeSensors == 3) {
        if (!stopLineArmed) {
            // First tick all three are ON -> start the confirmation timer
            stopLineArmed  = true;
            stopLineSeenAt = millis();
            Serial.println("------ Potential Stop Line ------");
        }
        driveMotors(BASE_SPEED / 2, BASE_SPEED / 2);

        if (millis() - stopLineSeenAt >= STOP_CONFIRM_MS) {
            // Stop line is genuine, transition to STOP_CONFIRM
            stopMotors();
            state = STOP_CONFIRM;
            Serial.println("------ STOP LINE CONFIRMED ------");
        }
        return; // Skip normal PID logic while confirming
    }

    // Cancel stop line confimation if sensors drop
    stopLineArmed = false;

    // ------- Normal PID Steering -------
    if (activeSensors > 0) {
        // Line is visible -> reset the loss timer and compute PID error
        linePreviouslyLost = false;
        lineLostAt = 0;

        // Average: negative = line is left, positive = line is right
        pidInput = (s1 * SENSOR1_WEIGHT +
                    s2 * SENSOR2_WEIGHT +
                    s3 * SENSOR3_WEIGHT) / activeSensors;

        // Remember which side the line was last seen for recovery direction
        if (pidInput < 0) lastTurnDir = RECOVER_RIGHT;
        if (pidInput > 0) lastTurnDir = RECOVER_LEFT;

        // Run PID computation
        linePID.Compute();

        // Convert PID correction into motor speeds:
        //   positive pidOutput -> line is right -> slow left, speed right
        //   negative pidOutput -> line is left  -> speed left, slow right
        int leftSpeed  = constrain((int)(BASE_SPEED - pidOutput), -MAX_SPEED, MAX_SPEED);
        int rightSpeed = constrain((int)(BASE_SPEED + pidOutput), -MAX_SPEED, MAX_SPEED);

        // Enfore miniumum speed
        if (leftSpeed  > 0) leftSpeed  = max(leftSpeed,  MIN_SPEED);
        if (rightSpeed > 0) rightSpeed = max(rightSpeed, MIN_SPEED);

        driveMotors(leftSpeed, rightSpeed);

        // Debug output
        Serial.print("S1:"); Serial.print(s1);
        Serial.print(" S2:"); Serial.print(s2);
        Serial.print(" S3:"); Serial.print(s3);
        Serial.print(" Err:"); Serial.print(pidInput, 3);
        Serial.print(" PID:"); Serial.print(pidOutput, 1);
        Serial.print(" L:"); Serial.print(leftSpeed);
        Serial.print(" R:"); Serial.println(rightSpeed);
    } 
    // ------- Line Loss Checker -------
    // All sensors are OFF, hold the last PID output
    // for LOSS_MS to bridge short gaps
    else {
        if (!linePreviouslyLost) {
            // First tick with no line –> start the loss timer
            linePreviouslyLost = true;
            lineLostAt = millis();
        }

        if (millis() - lineLostAt < LOSS_MS) {
            // Gap is short: keep going with last known correction
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

// ------ Stop Confirm State ------
//  Brief pause after the stop line is confirmed, give rover
// time to settle before going to pixy
void runStopConfirm() {
    stopMotors();
    delay(300);
    state = APPROACH_CAN;
    Serial.println("------ APPROACH CAN ------");
}

// ------- Approach Can State -------
// Uses the Pixy camera to locate the red can and drive toward it
// Transitions to GRAB_CAN once the can's block width reaches PIXY_GRAB_WIDTH
void runApproachCan() {
    pixy.ccc.getBlocks();   // Request latest colour block detections from Pixy

    // No blocks detected –> spin slowly to search for the can
    if (pixy.ccc.numBlocks == 0) {
        Serial.println("Can not found, searching...");
        driveMotors(-PIXY_TURN_SPEED, PIXY_TURN_SPEED);   // spin left to search
        return;
    }

    // Use the largest detected block
    int obj_x = pixy.ccc.blocks[0].m_x;
    int obj_width = pixy.ccc.blocks[0].m_width;

    // Pixel error: negative = can is left
    //              positive = can is right
    int error = obj_x - PIXY_CENTER_X;
    // Limit turn correction to max available value
    int turn_speed = constrain((int)(abs(error) * Kp_vision),
                                0, MAX_SPEED - APPROACH_SPEED);

    // Debug output
    Serial.print("Pixy x:"); 
    Serial.print(obj_x);
    Serial.print(" w:");     
    Serial.print(obj_width);
    Serial.print(" err:");   
    Serial.println(error);

    // Check if can is close enough to grab
    if (obj_width >= PIXY_GRAB_WIDTH) {
        stopMotors();
        state = GRAB_CAN;
        Serial.println("------ GRAB CAN ------");
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

// ------- Grab Can State -------
// Final state. Stops the rover, closes the claw around it
// then raises the arm.  Loops forever afterwards
void runGrabCan() {
    stopMotors();
    clawLife.write(CLAW_DOWN);      // Lower arm to grab can
    delay(250);                     // Settle before activating servo
    clawClose.write(CLAW_CLOSED);   // Close around the can
    delay(100);                     // Wait to close fully
    clawLift.write(CLAW_UP);        // Raise to lift the can
    Serial.println("------ CAN GRABBED ------");

    // Hold forever
    while (true) { 
        delay(100);
        clawLift.write(CLAW_UP);
    }
}

// ------- Line Recovery -------
// Called when the line has been lost for longer than LOSS_MS
// Spins the rover in the direction the line was last seen to
// search for it again.
void recoverLine() {
    Serial.println("LINE LOST");
    if (lastTurnDir == RECOVER_LEFT) {
        driveMotors(-RECOVER_SPEED,  RECOVER_SPEED);    // Spin left
    } 
    else if (lastTurnDir == RECOVER_RIGHT) {
        driveMotors( RECOVER_SPEED, -RECOVER_SPEED);    // Spin right
    } 
    else {
        stopMotors();   // No last known direction -> stop completly
    }
}

// ------- Motor Helpers -------
// Drives both motors at independent speeds
// Positive values = forward
// Left motor is negated to account for its reversed physical mounting
void driveMotors(int leftSpeed, int rightSpeed) {
    leftMotor.setSpeed(-leftSpeed);    // left motor mechanically reversed
    rightMotor.setSpeed(rightSpeed);
}
// Cut power to both motors
void stopMotors() {
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);
}