package org.firstinspires.ftc.teamcode.pollen;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

/** Autonomous never uses the flower drainer. When an OpMode ends, the SDK stops powering its
 * servos, so an auto that never commands the drainer leaves it limp and the arm drops. Every
 * auto calls holdUp() at INIT: the drainer goes to the TeleOp default UP position and stays
 * there for the whole run. A robot without the servo still runs its auto normally. */
public final class PollenDrainerHold {
    private PollenDrainerHold() { }

    public static final String SERVO_NAME = "pollenDrainer";

    /** @return a short status for telemetry/logs; never throws. */
    public static String holdUp(HardwareMap hardwareMap) {
        String status;
        try {
            Servo drainer = hardwareMap.get(Servo.class, SERVO_NAME);
            drainer.setPosition(PollenTeleOpTuning.DEFAULT_DRAINER_UP);
            status = "held UP at " + PollenTeleOpTuning.DEFAULT_DRAINER_UP;
        } catch (RuntimeException ex) {
            status = "not held (" + ex.getMessage() + ")";
        }
        RobotLog.ii("PollenDrainer", "Auto drainer %s", status);
        return status;
    }
}
