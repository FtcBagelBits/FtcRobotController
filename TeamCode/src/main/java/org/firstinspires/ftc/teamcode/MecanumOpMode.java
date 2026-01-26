package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class MecanumOpMode extends OpMode {
    private Launcher launcher = new Launcher();
    private MecanumDriveTrain driveTrain = new MecanumDriveTrain();
    private Auto auto = new Auto();
    private static final String TELEOP = "TELEOP";
    private static final String AUTO_BLUE_GOAL = "AUTO BLUE GOAL";
    private static final String AUTO_RED_GOAL = " AUTO RED GOAL";
    private static final String AUTO_BLUE_WALL = "AUTO BLUE WALL";
    private static final String AUTO_RED_WALL = "AUTO RED WALL";
    private String operationSelected = TELEOP;

    @Override
    public void init() {
        driveTrain.init(hardwareMap);
        launcher.init(gamepad1, telemetry);
    }

    @Override
    public void loop() {
        if (operationSelected.equals(AUTO_BLUE_GOAL)) {
            auto.doAutoBlueGoal();
        } else if (operationSelected.equals(AUTO_RED_GOAL)) {
            auto.doAutoRedGoal();
        } else if (operationSelected.equals(AUTO_BLUE_WALL)) {
            auto.doAutoBlueWall();
        }  else if (operationSelected.equals(AUTO_RED_WALL)) {
            auto.doAutoRedWall();
        } else {
            doTeleOp();
        }
    }
    private String selectOperation(String state, boolean cycleNext) {
        if (cycleNext) {
            if (state.equals(TELEOP)) {
                state = AUTO_BLUE_GOAL;
            } else if (state.equals(AUTO_BLUE_GOAL)) {
                state = AUTO_RED_GOAL;
            } else if (state.equals(AUTO_RED_GOAL)) {
                state = AUTO_BLUE_WALL;
            } else if (state.equals (AUTO_BLUE_WALL)) {
                state = AUTO_RED_WALL;
            } else if (state.equals (AUTO_RED_WALL)) {
                state = TELEOP;
            } else {
                telemetry.addData("WARNING", "Unknown Operation State Reached - Restart Program");
            }
        }
        telemetry.addLine("Press Home Button to cycle options");
        telemetry.addData("CURRENT SELECTION", state);
        if (state.equals(AUTO_BLUE_GOAL) || state.equals(AUTO_RED_GOAL) || state.equals(AUTO_BLUE_WALL) || state.equals(AUTO_RED_WALL)) {
            telemetry.addLine("Please remember to enable the AUTO timer!");
        }
        telemetry.addLine("Press START to start your program");
        telemetry.update();
        return state;
    }
    private void doTeleOp() {
        driveTrain.driveWithGamepad(gamepad1);
        launcher.shotButtons();
        launcher.manualFeederControl();
    }

    private void doAutoBlueGoal() {

    }

    private void doAutoRedGoal() {

    }

    private void doAutoBlueWall() {

    }

    private void doAutoRedWall() {

    }

}
