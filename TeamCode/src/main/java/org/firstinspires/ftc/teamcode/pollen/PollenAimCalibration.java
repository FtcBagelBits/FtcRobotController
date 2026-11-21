package org.firstinspires.ftc.teamcode.pollen;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BioBuzzStarterbotOpMode;

/**
 * CALIBRATION for Circle's aim (v36). PollenBot TeleOp, plus:
 *  - INIT with the robot at the PRIMARY shooter start (front flat on the end wall, lined up with
 *    our hive cell): the Pinpoint is zeroed there, so odometry knows where the hive is.
 *  - Drive anywhere, press Circle: it turns to aim. D-pad LEFT = aim with the Limelight (as in
 *    PollenBot), RIGHT = aim with odometry only.
 *  - The screen shows what the camera and odometry each think, live. Every Circle writes AIM rows
 *    (both estimates, the turn, and how far off the camera says it still is) to the TeleOp
 *    recording, to compare the two and tune the camera mount numbers.
 * D-pad UP/DOWN still nudge the drainer, as in PollenBot.
 */
@TeleOp(name = "Calibration Hive Aim", group = "4 Calibration")
public final class PollenAimCalibration extends BioBuzzStarterbotOpMode {
    @Override protected boolean aimCalibration() { return true; }
}
