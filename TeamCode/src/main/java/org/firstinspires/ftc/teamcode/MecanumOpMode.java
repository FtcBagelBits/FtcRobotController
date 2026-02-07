package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


//If we lose it's KIYO's fault. ALSO ALL OF OUR LITTLE SIBLINGS FAULTS
@TeleOp
public class MecanumOpMode extends LinearOpMode {
    private final Launcher launcher = new Launcher(this);
    public final MecanumDriveTrain driveTrain = new MecanumDriveTrain(this);
    private final Auto auto = new Auto(driveTrain, launcher);
    public final Camera camera = new Camera(this);
    private static final String TELEOP_BLUE_GOAL = "TELEOP_BLUE_GOAL";
    private static final String AUTO_BLUE_GOAL = "AUTO BLUE GOAL";
    private static final String AUTO_RED_GOAL = " AUTO RED GOAL";
    private static final String AUTO_BLUE_WALL = "AUTO BLUE WALL";
    private static final String AUTO_RED_WALL = "AUTO RED WALL";
    private static final String TELEOP_RED_GOAL = "TELEOP_RED_GOAL";
    public int goalId;

    private String operationSelected = TELEOP_BLUE_GOAL;
    private static final int BLUE_GOAL_ID = 20;
    private static final int RED_GOAL_ID = 24;

    @Override
    public void runOpMode() {

        // Init
        driveTrain.init(hardwareMap, telemetry);
        launcher.init(hardwareMap, gamepad1, telemetry);
        camera.init();

        // On initilization the Driver Station will prompt for which OpMode should be run - Auto Blue, Auto Red, or TeleOp
        while (opModeInInit()) {
            operationSelected = selectOperation(operationSelected, gamepad1.psWasPressed());
            telemetry.update();
        }

        waitForStart();

        if (operationSelected.equals(AUTO_BLUE_GOAL)) {
            this.goalId = BLUE_GOAL_ID;
            auto.doAutoBlueGoal();
        } else if (operationSelected.equals(AUTO_RED_GOAL)) {
            this.goalId = RED_GOAL_ID;
            auto.doAutoRedGoal();
        } else if (operationSelected.equals(AUTO_BLUE_WALL)) {
            this.goalId = BLUE_GOAL_ID;
            auto.doAutoBlueWall();
        } else if (operationSelected.equals(AUTO_RED_WALL)) {
            this.goalId = RED_GOAL_ID;
            auto.doAutoRedWall();
        } else if (operationSelected.equals(TELEOP_RED_GOAL)) {
            doTeleOp(RED_GOAL_ID);
        } else {
            doTeleOp(BLUE_GOAL_ID);
        }
    }

    private String selectOperation(String state, boolean cycleNext) {
        if (cycleNext) {
            if (state.equals(TELEOP_BLUE_GOAL)) {
                state = AUTO_BLUE_GOAL;
            } else if (state.equals(AUTO_BLUE_GOAL)) {
                state = AUTO_RED_GOAL;
            } else if (state.equals(AUTO_RED_GOAL)) {
                state = AUTO_BLUE_WALL;
            } else if (state.equals(AUTO_BLUE_WALL)) {
                state = AUTO_RED_WALL;
            } else if (state.equals(AUTO_RED_WALL)) {
                state = MecanumOpMode.TELEOP_RED_GOAL;
            } else if (state.equals(MecanumOpMode.TELEOP_RED_GOAL)) {
                state = TELEOP_BLUE_GOAL;
            } else {
                telemetry.addData("WARNING", "Unknown Operation State Reached - Restart Program");
            }
        }

        telemetry.addLine("Press Home Button to cycle options");
        telemetry.addData("CURRENT SELECTION", state);
        if (state.equals(AUTO_BLUE_GOAL) || state.equals(AUTO_RED_GOAL) || state.equals(AUTO_BLUE_WALL) || state.equals(AUTO_RED_WALL)) {
            telemetry.addLine("Please remember to enable the AUTO timer!");
        }
        telemetry.addLine("Press START to start your program, also if we lose its NOBODY's FAULT (yeah everyone lets beat up nobody) :)");
        telemetry.update();
        return state;
    }

    private void doTeleOp(int goalId) {
        this.goalId = goalId;

        while (opModeIsActive()) {
            driveTrain.driveWithGamepad(gamepad1);
            launcher.shotButtons();
            launcher.manualFeederControl();

        }
    }
}

