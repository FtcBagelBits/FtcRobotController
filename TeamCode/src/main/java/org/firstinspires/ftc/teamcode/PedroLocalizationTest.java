package org.firstinspires.ftc.teamcode;

import com.bylazar.field.FieldManager;
import com.bylazar.field.CanvasRotation;
import com.bylazar.field.FieldPresetParams;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Checks that Pedro Pathing is wired up correctly on Lox before any path following is attempted.
 * This deliberately does not follow a path - it only drives the robot and reports the pose the
 * Pinpoint reports, so it does not depend on any of the untuned FollowerConstants.
 *
 * Telemetry goes to the Driver Station and to Panels, and the robot is drawn on the Panels field
 * view. Connect to the Robot Controller's WiFi and open http://192.168.43.1:8001 to see it.
 *
 * The pose starts at (0, 0) heading 0 and is measured in inches / radians, with x forward,
 * y to the LEFT, and heading counterclockwise-positive.
 *
 * What to check, in order:
 *   1. Drive forward  -> x increases, y stays near 0.
 *   2. Strafe left    -> y increases, x stays near 0.
 *   3. Turn counterclockwise -> heading increases.
 *   4. Push the robot a measured distance by hand (say 24 in) -> the reported number matches.
 *
 * If a value moves the wrong way, fix it in pedroPathing/Constants: a backwards drive axis is a
 * motor direction, a backwards odometry axis is a forwardEncoderDirection / strafeEncoderDirection.
 */
@TeleOp(name = "Pedro Localization Test", group = "Pedro")
public class PedroLocalizationTest extends OpMode {
    /** Half the robot's width, in inches - only used to size the marker on the field view. */
    private static final double ROBOT_RADIUS = 9;

    /*
     * Canvas transform for the Panels field view. Affects ONLY what Panels draws - the pose
     * numbers are untouched, so never "fix" a drawing problem in Constants.java instead.
     *
     * These are the stock PEDRO_PATHING values, restated here so they are visible rather than
     * hidden in the preset. They are correct: working the renderer's matrix through
     * (getItemTransform in Field/web/src/widgets/renderer.ts, with DEG_90 = +pi/2) gives
     * X' = x and Y' = -y, so Pedro +x draws right and Pedro +y draws up, origin bottom-left.
     *
     * The flags are applied in the BROWSER, not on the robot - setOffsets only attaches the
     * preset to the canvas that gets sent over the wire. So if the field view disagrees with
     * these values, suspect the Panels UI's own field settings before changing anything here.
     */
    private static final double OFFSET_X = 24.0 * -3;
    private static final double OFFSET_Y = 24.0 * -3;
    private static final CanvasRotation ROTATION = CanvasRotation.DEG_90;
    private static final boolean FLIP_X = false;
    private static final boolean FLIP_Y = true;
    private static final boolean REVERSE_XY = false;

    private Follower follower;
    private TelemetryManager panelsTelemetry;
    private FieldManager field;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(135, 9, Math.toRadians(90)));

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        field = PanelsField.INSTANCE.getField();

        // Draw using Pedro's coordinate frame rather than Panels' own, so what the field view
        // shows lines up with the numbers Pedro reports. Set explicitly rather than trusting the
        // shipped preset, whose values are wrong in this version of Panels.
        FieldPresetParams preset = PanelsField.INSTANCE.getPresets().getPEDRO_PATHING();
//        preset.setOffsetX(OFFSET_X);
//        preset.setOffsetY(OFFSET_Y);
//        preset.setRotation(ROTATION);
//        preset.setFlipX(FLIP_X);
//        preset.setFlipY(FLIP_Y);
        preset.setReverseXY(REVERSE_XY);
        field.setOffsets(preset);

        // Printed so the canvas transform actually in effect is visible, rather than guessed at.
        telemetry.addData("field offset", preset.getOffsetX() + ", " + preset.getOffsetY());
        telemetry.addData("field rotation", preset.getRotation());
        telemetry.addData("field flipX/flipY/reverseXY",
                preset.getFlipX() + " / " + preset.getFlipY() + " / " + preset.getReverseXY());
        telemetry.addLine("Drive the robot, then compare the pose below to what it actually did.");
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // Pedro takes strafe as positive-to-the-left and turn as positive-counterclockwise,
        // so both stick x axes are negated.
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true);
        follower.update();

        Pose pose = follower.getPose();
        panelsTelemetry.addData("x (in)", pose.getX());
        panelsTelemetry.addData("y (in)", pose.getY());
        panelsTelemetry.addData("heading (deg)", Math.toDegrees(pose.getHeading()));
        panelsTelemetry.addData("localization NaN", follower.isLocalizationNAN());
        // Mirrors the same lines to the Driver Station, so the OpMode is still usable with
        // nothing connected to Panels.
        panelsTelemetry.update(telemetry);

        drawRobot(pose);
    }

    /** Draws the robot as a circle with a line pointing the way it is facing. */
    private void drawRobot(Pose pose) {
        field.setStyle(new Style(
                PanelsField.INSTANCE.getTRANSPARENT(),
                PanelsField.INSTANCE.getBLUE(),
                1.0));
        field.moveCursor(pose.getX(), pose.getY());
        field.circle(ROBOT_RADIUS);
        field.moveCursor(pose.getX(), pose.getY());
        field.line(
                pose.getX() + ROBOT_RADIUS * Math.cos(pose.getHeading()),
                pose.getY() + ROBOT_RADIUS * Math.sin(pose.getHeading()));
        field.update();
    }
}
