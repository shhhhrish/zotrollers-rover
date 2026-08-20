# Zotrollers Rover

Autonomous line-following rover built for **ENGR 7B (Winter 2026)**. The rover follows a black tape line using three IR sensors and a PID controller, detects a horizontal stop line, then switches to vision-guided navigation with a Pixy2 camera to locate, approach, and grab a red can with a two-servo claw.

## How It Works

The rover runs as a simple state machine with four states:

| State | Description |
|---|---|
| `LINE_FOLLOW` | Follows the tape line using a PID controller fed by three IR sensors. |
| `STOP_CONFIRM` | Brief pause to make sure stop line is confirmed and not a misreading on the line path |
| `APPROACH_CAN` | Uses the Pixy2 camera to locate the red can and drive toward it. |
| `GRAB_CAN` | Closes the claw around the can and lifts it. Final state — holds forever. |

### Line Following
Three IR sensors (left, center, right) are each sampled multiple times per loop and debounced with a threshold vote to filter noise. Active sensors are combined into a weighted error value (negative = line is left, positive = line is right) and fed into a PID controller, whose output differentially adjusts the left/right motor speeds to keep the rover centered on the line.

If all sensors momentarily read off the line, the rover holds its last known correction for a short grace period to bridge small gaps in the tape. If the line is lost longer than that, it enters a recovery spin toward the side the line was last seen on.

### Stop Line Detection
A stop line is detected when all three IR sensors read ON at the same time. This must hold for a short confirmation window to avoid false triggers from noise or line intersections before the rover stops and transitions to vision navigation.

### Can Approach & Grab
Once in vision mode, the Pixy2 reports the largest detected color block (the red can). The rover steers toward the can's horizontal offset from center while driving forward, using a proportional turn correction. When the can's detected width crosses a threshold, the rover stops, closes its claw, and lifts the arm.

## Hardware

- **Motor driver:** Cytron motor driver (PWM/DIR mode) — left motor on pins 6/7, right motor on pins 3/2
- **Line sensors:** 3x IR sensors on analog pins A0 (left), A1 (center), A2 (right)
- **Vision:** Pixy2 camera (color block detection)
- **Claw:** 2x servos — one for opening/closing the claw (pin 10), one for raising/lowering the arm (pin 4)

## Libraries

- [`CytronMotorDriver`](https://github.com/CytronTechnologies/CytronMotorDriver) — drive motor control
- [`PID_v1`](https://github.com/br3ttb/Arduino-PID-Library) — PID controller
- [`Pixy2`](https://github.com/charmedlabs/pixy2) — Pixy2 camera communication
- `Servo` — claw servo control (Arduino built-in)

## Repo Structure

```
zotrollers-rovercode/
└── zotrollers-rovercode.ino   # Main Arduino sketch
```

## Tuning

Key constants live at the top of `zotrollers-rovercode.ino`:

- **PID gains:** `Kp`, `Ki`, `Kd` — steering response for line following
- **Speeds:** `BASE_SPEED`, `MAX_SPEED`, `MIN_SPEED`, `RECOVER_SPEED`, `APPROACH_SPEED`, `PIXY_TURN_SPEED`
- **Sensor debounce:** `SAMPLE_COUNT`, `SAMPLE_THRESHOLD`
- **Stop line confirmation:** `STOP_CONFIRM_MS`
- **Line loss grace period:** `LOSS_MS`
- **Vision:** `PIXY_CENTER_X`, `PIXY_TURN_THRESH`, `PIXY_GRAB_WIDTH`, `Kp_vision`
- **Claw angles:** `CLAW_OPEN`, `CLAW_CLOSED`, `CLAW_DOWN`, `CLAW_UP`
