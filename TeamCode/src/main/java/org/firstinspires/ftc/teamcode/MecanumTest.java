package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pollen.PollenConfig;

/*
 * MecanumTest: drives the four mecanum wheels and nothing else.
 *
 * It never asks for the launcher, intake, agitator, camera or odometry, so it still runs
 * when those devices (or the Expansion Hub they are plugged into) are missing. It uses the
 * same motor names and directions as PollenBot (PollenConfig), so if this drives correctly,
 * the PollenBot drivetrain mapping is correct too.
 *
 * Driving (gamepad 1):
 *   Left stick  = forward/back and strafe
 *   Right stick = turn
 *   Hold left trigger = slow mode
 *
 * Wheel check (gamepad 1): hold one button to spin ONE wheel forward at low power.
 * Every wheel should push the robot FORWARD. If one spins backward, its direction in
 * PollenConfig.DRIVE_REVERSED is wrong. If the wrong wheel spins, its name is wrong
 * in the Robot Controller configuration.
 *   Square   = frontLeft     Triangle = frontRight
 *   Cross    = backLeft      Circle   = backRight
 */
@TeleOp(name = "MecanumTest", group = "Test")
public class MecanumTest extends OpMode {

    private static final double SLOW_SPEED = 0.3;      // same slow speed as our other mecanum code
    private static final double WHEEL_TEST_POWER = 0.3;

    // Same order as PollenConfig.DRIVE_NAMES: frontLeft, frontRight, backLeft, backRight
    private static final int FRONT_LEFT = 0;
    private static final int FRONT_RIGHT = 1;
    private static final int BACK_LEFT = 2;
    private static final int BACK_RIGHT = 3;

    private final DcMotor[] drive = new DcMotor[4];

    @Override
    public void init() {
        for (int i = 0; i < drive.length; i++) {
            // tryGet returns null instead of crashing, so a missing motor is reported below.
            drive[i] = hardwareMap.tryGet(DcMotor.class, PollenConfig.DRIVE_NAMES[i]);
            if (drive[i] == null) continue;
            drive[i].setDirection(PollenConfig.DRIVE_REVERSED[i]
                    ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            drive[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            drive[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            drive[i].setPower(0);
        }
    }

    @Override
    public void init_loop() {
        showMotors();
        telemetry.addLine("Drivetrain only: launcher, intake and agitator are not used.");
    }

    @Override
    public void loop() {
        double[] power = new double[4];

        if (gamepad1.square || gamepad1.triangle || gamepad1.cross || gamepad1.circle) {
            // Wheel check: one wheel at a time.
            if (gamepad1.square)   power[FRONT_LEFT]  = WHEEL_TEST_POWER;
            if (gamepad1.triangle) power[FRONT_RIGHT] = WHEEL_TEST_POWER;
            if (gamepad1.cross)    power[BACK_LEFT]   = WHEEL_TEST_POWER;
            if (gamepad1.circle)   power[BACK_RIGHT]  = WHEEL_TEST_POWER;
            telemetry.addLine("WHEEL CHECK: each wheel should push the robot forward");
        } else {
            double forward = -gamepad1.left_stick_y;   // stick up reads negative, so flip it
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            power[FRONT_LEFT]  = forward + strafe + turn;
            power[FRONT_RIGHT] = forward - strafe - turn;
            power[BACK_LEFT]   = forward - strafe + turn;
            power[BACK_RIGHT]  = forward + strafe - turn;

            // If any wheel is asked for more than 100%, scale all four down together
            // so the robot still moves in the direction the stick points.
            double max = 1.0;
            for (double p : power) max = Math.max(max, Math.abs(p));
            double scale = (gamepad1.left_trigger > 0 ? SLOW_SPEED : 1.0) / max;
            for (int i = 0; i < power.length; i++) power[i] *= scale;

            telemetry.addData("Mode", gamepad1.left_trigger > 0 ? "SLOW" : "full speed");
        }

        for (int i = 0; i < drive.length; i++) {
            if (drive[i] != null) drive[i].setPower(power[i]);
            telemetry.addData(PollenConfig.DRIVE_NAMES[i] + " power", "%.2f", power[i]);
        }
        showMotors();
    }

    /** Shows each drive motor and where the SDK found it, or MISSING. */
    private void showMotors() {
        for (int i = 0; i < drive.length; i++) {
            String name = PollenConfig.DRIVE_NAMES[i];
            if (drive[i] == null) {
                telemetry.addData(name, "MISSING - not in the active configuration");
            } else {
                telemetry.addData(name, (PollenConfig.DRIVE_REVERSED[i] ? "REVERSE, " : "FORWARD, ")
                        + drive[i].getConnectionInfo());
            }
        }
    }
}
