package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "AkiMecanumTeleOp")
public class AkiMecanumTeleOp extends LinearOpMode {

    //        private DcMotor flywheel;
//        private DcMotor coreHex;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontRight;
//        private CRServo servo;

    int BANK_VELOCITY;
    int FAR_VELOCITY;
    int MAX_VELOCITY;

    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     */
    @Override
    public void runOpMode() {
//            flywheel = hardwareMap.get(DcMotor.class, "flywheelAsDcMotor");
//            coreHex = hardwareMap.get(DcMotor.class, "coreHexAsDcMotor");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
//            servo = hardwareMap.get(CRServo.class, "servoAsCRServo");

        // Establishing the direction and mode for the motors
//            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            flywheel.setDirection(DcMotor.Direction.REVERSE);
//            coreHex.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
//            servo.setPower(0);
        // Setting our velocity targets. These values are in ticks per second!
        BANK_VELOCITY = 1300;
        FAR_VELOCITY = 1500;
        MAX_VELOCITY = 2200;
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Calling our functions while the OpMode is running
                mecanumDrive();
//                    setFlywheelVelocity();
//                    manualCoreHexAndServoControl();
//                    telemetry.addData("Flywheel Velocity", ((DcMotorEx) flywheel).getVelocity());
//                    telemetry.addData("Flywheel Power", flywheel.getPower());
//                    telemetry.update();
            }
        }
    }

    /**
     * Describe this function...
     */
//        private void manualCoreHexAndServoControl() {
//            // Manual control for the Core Hex intake
//            if (gamepad1.cross) {
//                coreHex.setPower(0.5);
//            } else if (gamepad1.triangle) {
//                coreHex.setPower(-0.5);
//            }
//            // Manual control for the hopper's servo
//            if (gamepad1.dpad_left) {
//                servo.setPower(1);
//            } else if (gamepad1.dpad_right) {
//                servo.setPower(-1);
//            }
//        }

    /**
     * Describe this function...
     */
//        private void bankShotAuto() {
//            ((DcMotorEx) flywheel).setVelocity(BANK_VELOCITY);
//            servo.setPower(-1);
//            if (((DcMotorEx) flywheel).getVelocity() >= BANK_VELOCITY - 100) {
//                coreHex.setPower(1);
//            } else {
//                coreHex.setPower(0);
//            }
//        }

    /**
     * Describe this function...
     */
//        private void farPowerAuto() {
//            ((DcMotorEx) flywheel).setVelocity(FAR_VELOCITY);
//            servo.setPower(-1);
//            if (((DcMotorEx) flywheel).getVelocity() >= FAR_VELOCITY - 100) {
//                coreHex.setPower(1);
//            } else {
//                coreHex.setPower(0);
//            }
//        }

    /**
     * Describe this function...
     */
//        private void setFlywheelVelocity() {
//            if (gamepad1.options) {
//                flywheel.setPower(-0.5);
//            } else if (gamepad1.left_bumper) {
//                farPowerAuto();
//            } else if (gamepad1.right_bumper) {
//                bankShotAuto();
//            } else if (gamepad1.circle) {
//                ((DcMotorEx) flywheel).setVelocity(BANK_VELOCITY);
//            } else if (gamepad1.square) {
//                ((DcMotorEx) flywheel).setVelocity(MAX_VELOCITY);
//            } else {
//                ((DcMotorEx) flywheel).setVelocity(0);
//                coreHex.setPower(0);
//                // The check below is in place to prevent stuttering with the servo. It checks if the servo is under manual control!
//                if (!gamepad1.dpad_right && !gamepad1.dpad_left) {
//                    servo.setPower(0);
//                }
//            }
//        }

    /**
     * Describe this function...
     */
    private void mecanumDrive() {
        float forwardBack;
        float strafe;
        float turn;
        float leftFrontPower;
        float rightFrontPower;
        float leftBackPower;
        float rightBackPower;

        forwardBack = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x;
        leftFrontPower = forwardBack + strafe + turn;
        rightFrontPower = (forwardBack - strafe) - turn;
        leftBackPower = (forwardBack - strafe) + turn;
        rightBackPower = (forwardBack + strafe) - turn;
        frontLeft.setPower(leftFrontPower);
        frontRight.setPower(rightFrontPower);
        backLeft.setPower(leftBackPower);
        backRight.setPower(rightBackPower);
    }
}