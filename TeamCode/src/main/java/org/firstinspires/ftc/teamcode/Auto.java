package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Auto {
    private boolean isAutoStarted = false;
    private boolean isShotStarted = false;
    private final MecanumDriveTrain driveTrain;
    private final Launcher launcher;
    private final ElapsedTime shotTimer = new ElapsedTime();
    private final ElapsedTime driveTimer = new ElapsedTime();

    public Auto(MecanumDriveTrain driveTrain, Launcher launcher) {
        this.driveTrain = driveTrain;
        this.launcher = launcher;
    }

    public void doAutoBlueGoal() {

    }


    public void doAutoRedGoal() {

    }

    public void doAutoBlueWall() {
        drive(0,0,-1,0.3,450);
        farShot(5000);
    }

    public void doAutoRedWall() {
        drive(0,0,1,0.3,450);
        farShot(5000);
    }

    private void farShot(int timeout_ms) {
        if (isShotStarted == false) {
            shotTimer.reset();
            isShotStarted = true;
        }
        while (shotTimer.milliseconds() < timeout_ms) {
            launcher.farShot();
        }
    }
    private void drive(double forward, double strafe, double turn, double power, int timeout_ms) {
        if (isAutoStarted == false) {
            driveTimer.reset();
            isAutoStarted = true;
        }
        while (driveTimer.milliseconds() < timeout_ms) {
            driveTrain.drive(forward, strafe, turn, power);
        }
        driveTrain.drive(0, 0, 0, 0);
    }

}
