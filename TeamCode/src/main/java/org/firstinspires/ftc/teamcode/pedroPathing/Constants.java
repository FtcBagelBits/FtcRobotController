package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Pedro Pathing configuration for Lox (the BioBuzzOpMode robot).
 *
 * Build a Follower with:
 *     Follower follower = Constants.createFollower(hardwareMap);
 *
 * NOTHING IS TUNED YET. The drivetrain values below are Pedro's generic defaults and will
 * not track a path accurately until they are measured on this robot. See the comments on
 * followerConstants and mecanumConstants for which ones matter and in what order.
 */
public class Constants {

    /*
     * These are Pedro's stock values. The three that must be measured before paths will
     * follow correctly are mass (kg), forwardZeroPowerAcceleration and
     * lateralZeroPowerAcceleration; the PIDF coefficients are tuned after those.
     */
    public static FollowerConstants followerConstants = new FollowerConstants();

    /*
     * Motor names and directions are copied from MecanumDrive.java so that Pedro drives Lox
     * the same way the TeleOp does. If a motor is ever re-wired or re-configured, change it
     * in BOTH places - they are independent.
     *
     * xVelocity/yVelocity are Pedro's defaults (max forward/lateral speed in in/s). They are
     * measured with Pedro's forward and lateral tuners; until then, path timing will be off.
     */
    public static MecanumConstants mecanumConstants = new MecanumConstants()
            .leftFrontMotorName("frontLeft")
            .rightFrontMotorName("frontRight")
            .leftRearMotorName("backLeft")
            .rightRearMotorName("backRight")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);

    /*
     * Taken from the values already tuned in SensorGoBildaPinpoint:
     *     setOffsets(154.0, -81.0, DistanceUnit.MM)
     *     setEncoderResolution(goBILDA_4_BAR_POD)
     *     setEncoderDirections(FORWARD, FORWARD)
     *
     * Pedro passes forwardPodY and strafePodX straight through to the driver's
     * setOffsets(xOffset, yOffset, unit) in that order, so these map one-to-one despite the
     * confusing names: forwardPodY is how far LEFT of center the forward pod sits, and
     * strafePodX is how far FORWARD of center the strafe pod sits.
     *
     * The offsets are converted from the millimetres above into inches because distanceUnit is
     * NOT just for these offsets - PinpointLocalizer also uses it in setX/setY, which receive
     * inches from Pedro. Setting it to MM would make every non-zero starting pose be written to
     * the Pinpoint as millimetres. 154 mm = 6.062992 in, 81 mm = 3.188976 in.
     */
    public static PinpointConstants pinpointConstants = new PinpointConstants()
            .hardwareMapName("pinpoint")
            .distanceUnit(DistanceUnit.INCH)
            .forwardPodY(6.062992)
            .strafePodX(-3.188976)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(mecanumConstants)
                .pinpointLocalizer(pinpointConstants)
                .build();
    }
}
