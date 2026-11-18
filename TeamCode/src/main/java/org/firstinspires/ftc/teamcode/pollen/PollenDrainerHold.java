package org.firstinspires.ftc.teamcode.pollen;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

/** Hold the drainer up during driving and shooting autos. The dedicated flower practice
 * takes control of the servo itself so it can lower it with the intake running. */
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
