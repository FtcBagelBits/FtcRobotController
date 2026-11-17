package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * PollenDrainerTest
 *
 * Moves one servo between an UP position and a DOWN position.
 *
 * Controls (gamepad 1):
 *   D-pad UP    -> move up by STEP (one press = one step)
 *   D-pad DOWN  -> move down by STEP (one press = one step)
 *   A button    -> toggle between UP and DOWN (one press = one switch)
 *
 * Setup:
 *   In the Driver Station robot configuration, name the servo "armServo"
 *   (or change SERVO_NAME below to match your config).
 *
 * How servo positions work:
 *   A servo position is a number from 0.0 to 1.0.
 *   0.0 is one end of the servo's travel, 1.0 is the other end.
 *   Start with values near the middle, test, then adjust UP_POSITION and
 *   DOWN_POSITION so the mechanism doesn't hit anything.
 */
@TeleOp(name = "PollenDrainerTest", group = "Test")
public class PollenDrainerTest extends OpMode {

    // ---- Things you will probably want to tune ----
    private static final String SERVO_NAME    = "pollenDrainer";
    private static final double UP_POSITION   = 0.45;
    private static final double DOWN_POSITION = 0.02;
    private static final double STEP          = 0.01;

    private Servo armServo;

    // Tracks where we told the servo to go
    private double armPosition = UP_POSITION;
    private boolean isUp = false;

    // Used so holding a button only acts once, not every loop
    private boolean aWasPressed = false;
    private boolean upWasPressed = false;
    private boolean downWasPressed = false;

    // 1. Runs once when the driver presses INIT
    @Override
    public void init() {
        // Connect to the servo using its configuration name
        armServo = hardwareMap.get(Servo.class, SERVO_NAME);

        // If "up" goes the wrong way on your robot, uncomment this line:
        // armServo.setDirection(Servo.Direction.REVERSE);

        telemetry.addData("Status", "Ready - press PLAY");
    }

    // 2. Runs once when the driver presses PLAY
    //    (Servos don't move during INIT unless we tell them to.)
    @Override
    public void start() {
        // Start in the DOWN position
        armServo.setPosition(armPosition);
    }

    // 3. Main loop - runs over and over until STOP is pressed
    @Override
    public void loop() {

        // D-pad: nudge the position one STEP, only on the moment it is first pressed
        boolean upIsPressed = gamepad1.dpad_up;
        if (upIsPressed && !upWasPressed) {
            armPosition += STEP;
        }
        upWasPressed = upIsPressed;

        boolean downIsPressed = gamepad1.dpad_down;
        if (downIsPressed && !downWasPressed) {
            armPosition -= STEP;
        }
        downWasPressed = downIsPressed;

        // A button: toggle, but only on the moment it is first pressed
        boolean aIsPressed = gamepad1.a;
        if (aIsPressed && !aWasPressed) {
            isUp = !isUp;
            armPosition = isUp ? UP_POSITION : DOWN_POSITION;
        }
        aWasPressed = aIsPressed;

        // Keep the position inside the servo's 0.0 - 1.0 range
        armPosition = Math.max(0.0, Math.min(1.0, armPosition));

        // Send the servo to the chosen position
        armServo.setPosition(armPosition);

        // Show what's happening on the Driver Station
        telemetry.addData("Toggle state", isUp ? "UP" : "DOWN");
        telemetry.addData("Commanded position", "%.2f", armServo.getPosition());
        telemetry.addLine("D-pad up/down to step, A to toggle");
    }
}
