package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class MecanumOpMode extends OpMode {
    private Launcher launcher = new Launcher();
    private MecanumDriveTrain driveTrain = new MecanumDriveTrain();
    @Override
    public void init() {
        driveTrain.init(hardwareMap);
        launcher.init(gamepad1, telemetry);
    }

    @Override
    public void loop() {
driveTrain.driveWithGamepad();
launcher.shotButtons();
    }
}
