package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Auto {
    private final MecanumDriveTrain driveTrain;
    private final ElapsedTime driveTimer = new ElapsedTime();

    public Auto(MecanumDriveTrain driveTrain) {
        this.driveTrain = driveTrain;
    }

    public void doAutoBlueGoal() {

    }
//Positive Strafe makes the robot go to the Right.
    //Positive Forward makes the robot go Backwards
    //Positive Turn makes the robot turn to the Right

    public void doAutoRedGoal() { drive(0, 0,1, 0.3, 1000);
    }

    public void doAutoBlueWall() {
        drive(1, 0, 0, 0.3, 500);
    }

    public void doAutoRedWall() {

    }

    private void drive(double forward, double strafe, double turn, double power, int timeout_ms) {
        driveTimer.reset();
        while (driveTimer.milliseconds() < timeout_ms) {
            driveTrain.drive(forward, strafe, turn, power);
        }
        driveTrain.drive(0, 0, 0, 0);
    }

}
