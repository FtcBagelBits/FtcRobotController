package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "AkiMecanumTeleOp")
public class AkiMecanumTeleOp extends OpMode {
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontRight;
    private final float slowScaleFactor = 0.3F;
    private final boolean enableSlowMode = true;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void loop() {
        mecanumDrive();
    }

    /**
     *
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

        if (gamepad1.left_trigger > 0.0F || enableSlowMode) {
            leftFrontPower *= slowScaleFactor;
            rightFrontPower *= slowScaleFactor;
            leftBackPower *= slowScaleFactor;
            rightBackPower *= slowScaleFactor;
        }

        frontLeft.setPower(leftFrontPower);
        frontRight.setPower(rightFrontPower);
        backLeft.setPower(leftBackPower);
        backRight.setPower(rightBackPower);
    }
}