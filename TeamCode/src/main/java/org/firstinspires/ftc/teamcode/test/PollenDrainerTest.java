package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pollen.PollenConfig;
import org.firstinspires.ftc.teamcode.pollen.PollenTeleOpTuning;

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
 *   In the Driver Station robot configuration, name the servo "pollenDrainer"
 *   (or change SERVO_NAME below to match your config).
 *
 * How servo positions work:
 *   A servo position is a number from 0.0 to 1.0.
 *   0.0 is one end of the servo's travel, 1.0 is the other end.
 *   During INIT, select a row with D-pad UP/DOWN and edit with LEFT/RIGHT.
 *   Up/down positions change by 0.01; step changes by 0.002. Triangle resets
 *   defaults (0.45 up, 0.02 down, 0.01 step). START applies the up position.
 */
@TeleOp(name = "PollenDrainerTest", group = "Test")
public class PollenDrainerTest extends OpMode {

    // ---- Things you will probably want to tune ----
    private static final String SERVO_NAME    = "pollenDrainer";
    private static PollenTeleOpTuning.Snapshot remembered;
    private PollenTeleOpTuning tuning;

    private Servo armServo;

    // Tracks where we told the servo to go
    private double armPosition;
    private boolean isUp = true;

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

        tuning = PollenTeleOpTuning.drainerOnly(remembered);
        telemetry.addData("Status", "Ready - adjust drainer settings before PLAY");
    }

    @Override
    public void init_loop() {
        tuning.update(gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.dpad_left, gamepad1.dpad_right,
                gamepad1.triangle);
        remembered = tuning.snapshot();
        telemetry.addData("Build", PollenConfig.BUILD);
        telemetry.addData(marker(PollenTeleOpTuning.Row.DRAINER_UP) + "Drainer up position", "%.2f (+/- 0.01)", tuning.drainerUpPosition());
        telemetry.addData(marker(PollenTeleOpTuning.Row.DRAINER_DOWN) + "Drainer down position", "%.2f (+/- 0.01)", tuning.drainerDownPosition());
        telemetry.addData(marker(PollenTeleOpTuning.Row.DRAINER_STEP) + "Drainer step", "%.3f (+/- 0.002)", tuning.drainerStep());
        telemetry.addData("Controls", "D-pad UP/DOWN selects; LEFT/RIGHT edits; Triangle = defaults");
        telemetry.addData("Settings", tuning.isDefault() ? "DEFAULTS" : "CHANGED - check before START");
    }

    private String marker(PollenTeleOpTuning.Row row) {
        return tuning.selected() == row ? "> " : "  ";
    }

    // 2. Runs once when the driver presses PLAY
    //    (Servos don't move during INIT unless we tell them to.)
    @Override
    public void start() {
        tuning.lock();
        armPosition = tuning.drainerUpPosition();
        isUp = true;
        aWasPressed = gamepad1.a;
        upWasPressed = gamepad1.dpad_up;
        downWasPressed = gamepad1.dpad_down;
        armServo.setPosition(armPosition);
    }

    // 3. Main loop - runs over and over until STOP is pressed
    @Override
    public void loop() {

        // D-pad: nudge the position one STEP, only on the moment it is first pressed
        boolean upIsPressed = gamepad1.dpad_up;
        if (upIsPressed && !upWasPressed) {
            armPosition += tuning.drainerStep();
        }
        upWasPressed = upIsPressed;

        boolean downIsPressed = gamepad1.dpad_down;
        if (downIsPressed && !downWasPressed) {
            armPosition -= tuning.drainerStep();
        }
        downWasPressed = downIsPressed;

        // A button: toggle, but only on the moment it is first pressed
        boolean aIsPressed = gamepad1.a;
        if (aIsPressed && !aWasPressed) {
            isUp = !isUp;
            armPosition = isUp ? tuning.drainerUpPosition() : tuning.drainerDownPosition();
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
