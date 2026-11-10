/*   MIT License
 *   Copyright (c) [2026] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pollen.PollenConfig;
import org.firstinspires.ftc.teamcode.pollen.PollenTeleOpTuning;

/*
 * LauncherTest runs PollenBot's launcher, intake and agitator on their own, with the wheels off,
 * so the robot can sit on a table or a stand while you test a shot.
 *
 * It launches EXACTLY like PollenBot TeleOp (BioBuzzStarterbotOpMode) and like the autonomous
 * shot:
 *   - the same launcher speed, spin-up time and agitator power, from pollen/PollenConfig.java,
 *     with the same INIT menu as the TeleOp to change them;
 *   - the same feed rule, from pollen/PollenFeedRule.java: start feeding when the launcher is up
 *     to speed, or when the spin-up time is over, whichever happens first;
 *   - the same buttons: hold R1 to launch, hold L1 to run the launcher backwards and clear a
 *     jam, right trigger = intake in, left trigger = intake out.
 *
 * If you change launch() here or in the TeleOp, change the other one the same way.
 */

@TeleOp(name = "LauncherTest", group="Test")
//@Disabled
public class LauncherTestOpMode extends OpMode {

    // Declare OpMode members.
    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private DcMotor agitator = null;

    /*
     * The same numbers as PollenBot TeleOp, from the same place (pollen/PollenConfig.java).
     * Speeds are in encoder ticks per second: 28 ticks per turn, so 1180 ticks/s is 2529 RPM.
     */
    public final int LAUNCHER_TARGET_VELOCITY = PollenConfig.LAUNCHER_TARGET_VELOCITY; // 1180
    public final double LAUNCHER_SPINUP_SECONDS = PollenConfig.LAUNCHER_SPINUP_SECONDS; // 3 s
    public final int LAUNCHER_REVERSE_VELOCITY = PollenConfig.LAUNCHER_REVERSE_VELOCITY; // 140 = 300 RPM
    public final double INTAKE_FEED_BOOST = PollenConfig.INTAKE_FEED_BOOST; // 0.5

    // Create a variable to set to the intake.
    double intakePower;
    // The launcher's MEASURED speed in ticks per second, read ONCE in each loop.
    double launcherMeasuredVelocity;
    private boolean reversing = false;
    private final ElapsedTime launchTimer = new ElapsedTime();
    private boolean launching = false;
    private boolean feeding = false;   // latched on until R1 is released
    private String feedStartedBy = "";
    private double feedStartedAt = 0;

    /*
     * The same INIT menu as the TeleOp. LauncherTest keeps its own remembered settings, so a
     * speed you try here does not change the TeleOp until you enter it there too.
     */
    private static PollenTeleOpTuning.Snapshot remembered;
    private PollenTeleOpTuning tuning;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */
        agitator = hardwareMap.get(DcMotor.class, "agitator");
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "pollenLauncher");

        agitator.setDirection(DcMotor.Direction.REVERSE);
        launcher.setDirection(DcMotor.Direction.REVERSE);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting.
         */
        agitator.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        /*
         * Here we set our launcher to the RUN_USING_ENCODER runmode.
         * If you notice that you have no control over the velocity of the motor, it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         */

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));

        tuning = new PollenTeleOpTuning(LAUNCHER_TARGET_VELOCITY, LAUNCHER_SPINUP_SECONDS,
                PollenConfig.AGITATOR_FEED_POWER, remembered);

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        tuning.update(gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.dpad_left, gamepad1.dpad_right,
                gamepad1.triangle);
        remembered = tuning.snapshot();
        telemetry.addData("Build", PollenConfig.BUILD);
        telemetry.addData("Mode", "LauncherTest: launcher, intake and agitator only. The wheels stay off.");
        telemetry.addData(marker(PollenTeleOpTuning.Row.LAUNCHER) + "Launcher ticks/s", "%.0f (default %d)",
                tuning.launcherVelocity(), LAUNCHER_TARGET_VELOCITY);
        telemetry.addData(marker(PollenTeleOpTuning.Row.SPINUP) + "Spin-up before feed", "%.1f s (default %.1f s)",
                tuning.spinupSeconds(), LAUNCHER_SPINUP_SECONDS);
        telemetry.addData(marker(PollenTeleOpTuning.Row.AGITATOR) + "Agitator power while feeding", "%.2f (default %.2f)",
                tuning.agitatorPower(), PollenConfig.AGITATOR_FEED_POWER);
        // The menu is the TeleOp's, so it has a drive power row too. Nothing here uses it.
        telemetry.addData(marker(PollenTeleOpTuning.Row.DRIVE) + "Drive power", "not used in LauncherTest");
        telemetry.addData("Controls", "D-pad UP/DOWN selects; LEFT/RIGHT edits; Triangle = defaults");
    }

    private String marker(PollenTeleOpTuning.Row row) {
        return tuning.selected() == row ? "> " : "  ";
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        tuning.lock();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // Read the launcher's speed sensor (its encoder) once for this loop.
        launcherMeasuredVelocity = launcher.getVelocity();

        /*
         * Set the intake power variable to equal the right trigger, minus the left trigger.
         * Each trigger outputs a signal from 0-1, with 0 as fully released, and 1 fully depressed.
         * We use this technique (creating a variable, and setting it to our control inputs) to
         * allow us to avoid setting the same motors/servos power more than once per loop. That can
         * create erratic behavior.
         */
        intakePower = gamepad1.right_trigger - gamepad1.left_trigger;

        launch();

        // While L1 reverses the launcher, nothing may feed forward: a ball pushed into a wheel
        // spinning backwards jams again. The left trigger can still run the intake backwards.
        if (reversing) intakePower = Math.min(intakePower, 0);

        // R1 plus a full trigger would ask for 1.5; keep commands in the motors' -1..1 range.
        intakePower = Math.max(-1, Math.min(1, intakePower));
        // While feeding the launcher, the agitator runs at its own (full by default) power;
        // otherwise it follows the triggers.
        agitator.setPower(feeding ? tuning.agitatorPower() : intakePower);
        intake.setPower(intakePower);

        /*
         * Show what is happening on the Driver Station via telemetry.
         */
        telemetry.addData("Triggers", "left (%.2f), right (%.2f)", gamepad1.left_trigger, gamepad1.right_trigger);
        telemetry.addData("Launcher target / measured", "%.0f / %.0f ticks/s%s",
                tuning.launcherVelocity(), launcherMeasuredVelocity,
                launching && launcherMeasuredVelocity < 50 ? "  (measured ~0: check launcher encoder cable)" : "");
        telemetry.addData("Up to speed at", "%.0f ticks/s", tuning.launcherReadyVelocity());
        telemetry.addData("Feed", reversing ? "REVERSING launcher (L1) - no feed"
                : !launching ? "off (hold R1; L1 = reverse to clear a jam)"
                : feeding ? String.format("FEEDING (started at %.1f s: %s)", feedStartedAt, feedStartedBy)
                : "spinning up");
        telemetry.addData("Intake / agitator command", "%.2f / %.2f",
                intakePower, feeding ? tuning.agitatorPower() : intakePower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    /*
     * The same as launch() in the TeleOp (BioBuzzStarterbotOpMode). Keep the two the same.
     */
    void launch() {
        /*
         * L1 clears a jam: the launcher runs backwards at 300 RPM while L1 is held. L1 wins over
         * R1, and nothing feeds while reversing. Let go of L1 and R1 works as normal (with a
         * fresh spin-up).
         */
        reversing = gamepad1.left_bumper;
        if (reversing) {
            launching = false;
            launcher.setVelocity(-LAUNCHER_REVERSE_VELOCITY);
        } else if (gamepad1.right_bumper) {
            if (!launching) launchTimer.reset(); // R1 just pressed: start the spin-up clock
            launching = true;
            launcher.setVelocity(tuning.launcherVelocity());
        } else {
            launching = false;
            launcher.setVelocity(0);
        }

        /*
         * When do we start feeding balls into the launcher? Whichever happens FIRST:
         *   1. the encoder says the launcher is up to speed (96% of the target), or
         *   2. R1 has been held for the spin-up time (3 s), in case the encoder reading is wrong.
         * Once feeding starts, keep feeding until R1 is released. Each ball slows the wheel a
         * little, and we don't want that dip to switch the agitator off in the middle of a volley.
         */
        boolean upToSpeed = tuning.launcherIsUpToSpeed(launcherMeasuredVelocity);
        boolean spinUpTimeIsOver = tuning.spinUpTimeIsOver(launchTimer.seconds());
        if (!launching) {
            feeding = false;
        } else if (!feeding && (upToSpeed || spinUpTimeIsOver)) {
            feeding = true;
            feedStartedAt = launchTimer.seconds();
            feedStartedBy = upToSpeed ? "up to speed" : "spin-up timer";
        }
        if (feeding) {
            intakePower += INTAKE_FEED_BOOST;
        }
    }
}
