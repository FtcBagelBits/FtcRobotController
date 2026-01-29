package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MecanumDriveTrain {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private static final float slowSpeed = 0.3F;
    private final double WHEELS_INCHES_TO_TICKS = (28 * 5 * 3) / (3 * Math.PI);
    private final ElapsedTime autoDriveTimer = new ElapsedTime();

    public void init(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Correct way if motors are configured correctly
//        if (true) {
//            frontLeft.setDirection(DcMotor.Direction.REVERSE);
//            backLeft.setDirection(DcMotor.Direction.REVERSE);
//            frontRight.setDirection(DcMotor.Direction.FORWARD);
//            backRight.setDirection(DcMotor.Direction.FORWARD);
//        }

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void driveWithGamepad(Gamepad gamepad1) {
        double forwardBack = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        float power = 1;
        if (gamepad1.left_trigger > 0) {
            power = slowSpeed;
        }

        drive(forwardBack, strafe, turn, power);
    }

    public void drive(double forward, double strafe, double turn, double power) {
        double leftFrontPower = forward - strafe - turn;
        double rightFrontPower = forward + strafe + turn;
        double leftBackPower = forward + strafe - turn;
        double rightBackPower = forward - strafe + turn;
        // Setting Motor Power
        frontLeft.setPower(leftFrontPower * power);
        frontRight.setPower(rightFrontPower * power);
        backLeft.setPower(leftBackPower * power);
        backRight.setPower(rightBackPower * power);
    }

}