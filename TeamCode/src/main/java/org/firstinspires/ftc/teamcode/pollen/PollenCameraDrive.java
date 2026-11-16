package org.firstinspires.ftc.teamcode.pollen;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/** Slow, manual camera exploration. No autonomous aiming, feeding, calibration or odometry. */
@TeleOp(name = "TEST Camera Drive", group = "Pollen Test")
public final class PollenCameraDrive extends OpMode {
    private final DcMotor[] drive = new DcMotor[4];
    private final List<DcMotor> mechanisms = new ArrayList<>();
    private final PollenCameraDriveControl control = new PollenCameraDriveControl();
    private Limelight3A camera;
    private PollenCameraViewer viewer;
    private boolean started;
    private String error = "", viewerError = "";
    private long lastDisplay, lastLoop;
    private double loopMs;

    @Override public void init() {
        try {
            for (int i = 0; i < drive.length; i++) {
                drive[i] = hardwareMap.get(DcMotor.class, PollenConfig.DRIVE_NAMES[i]);
                drive[i].setPower(0);
            }
            List<String> names = new ArrayList<>();
            List<DcMotor> motors = new ArrayList<>();
            for (int i = 0; i < drive.length; i++) { names.add(PollenConfig.DRIVE_NAMES[i]); motors.add(drive[i]); }
            for (String name : new String[]{"pollenLauncher", "intake", "agitator"}) {
                DcMotor motor = hardwareMap.tryGet(DcMotor.class, name);
                if (motor != null) { mechanisms.add(motor); motor.setPower(0); names.add(name); motors.add(motor); }
            }
            PollenHiveHardware.requireDistinct(names.toArray(new String[0]), motors.toArray(new DcMotor[0]));
            for (int i = 0; i < drive.length; i++) {
                drive[i].setDirection(PollenConfig.DRIVE_REVERSED[i] ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
                drive[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                drive[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            camera = hardwareMap.get(Limelight3A.class, "limelight");
            camera.setPollRateHz(50);
            camera.start(); // Poll results; leave the camera's active pipeline and settings alone.
            try { viewer = new PollenCameraViewer(); }
            catch (Exception ex) { viewerError = "Viewer not started: " + ex.getMessage(); }
        } catch (Exception ex) {
            error = "INIT failed; driving disabled: " + ex.getMessage();
            stopMotors();
        }
        display(true);
    }

    @Override public void init_loop() { display(true); }

    @Override public void start() {
        stopMotors();
        control.reset();
        started = error.isEmpty();
        lastLoop = System.nanoTime();
    }

    @Override public void loop() {
        long now = System.nanoTime();
        loopMs = (now - lastLoop) / 1e6;
        lastLoop = now;
        if (started) {
            double[] powers = control.update(gamepad1.left_bumper, gamepad1.right_bumper,
                    -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            for (int i = 0; i < drive.length; i++) drive[i].setPower(powers[i]);
        }
        display(false);
    }

    private void display(boolean init) {
        long now = System.nanoTime();
        if (lastDisplay != 0 && now - lastDisplay < 250_000_000L) return;
        lastDisplay = now;
        telemetry.addData("BUILD", PollenConfig.BUILD);
        telemetry.addData("Mode", init ? "INIT: camera live; motors stopped" : "Manual camera drive; NO SHOOTING");
        telemetry.addLine("After START release L1 once. Hold L1 to drive; L1+R1 = fine.");
        telemetry.addLine("Left stick: forward/strafe. Right stick: turn. Release L1: stop.");
        telemetry.addData("Power cap", gamepad1.right_bumper ? "15%" : "30%");
        telemetry.addData("Browser (Hub Wi-Fi)", "http://192.168.43.1:" + PollenCameraViewer.PORT);
        telemetry.addData("Viewer", viewer == null ? viewerError : viewer.status());
        if (viewer != null) telemetry.addData("Video transferred", "%.2f MB", viewer.bytes() / 1e6);
        telemetry.addData("Loop interval", "%.1f ms (compare video open/paused)", loopMs);
        if (!error.isEmpty()) telemetry.addData("ERROR", error);
        LLResult result = camera == null ? null : camera.getLatestResult();
        if (result == null) {
            telemetry.addData("Camera", "No results yet; check USB and camera power");
        } else {
            long age = result.getStaleness();
            boolean fresh = age >= 0 && age <= 150;
            telemetry.addData("Camera", "%s; result age %d ms", fresh ? "FRESH" : "STALE", age);
            telemetry.addData("Active pipeline", "%d (%s); unchanged by this mode", result.getPipelineIndex(), result.getPipelineType());
            if (fresh && result.isValid()) {
                telemetry.addData("Target tx / ty", "%.1f / %.1f degrees", result.getTx(), result.getTy());
                StringBuilder tags = new StringBuilder();
                for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                    if (tags.length() > 0) tags.append("; ");
                    tags.append(String.format(Locale.US, "%d (%.1f, %.1f deg)", tag.getFiducialId(),
                            tag.getTargetXDegrees(), tag.getTargetYDegrees()));
                }
                telemetry.addData("AprilTags", tags.length() == 0 ? "none in this result" : tags.toString());
            } else telemetry.addData("Targets", fresh ? "none" : "stale: previous targets hidden");
        }
        telemetry.addLine("Raw camera observations only, not hive UP/DOWN readiness. Video may lag.");
        telemetry.update();
    }

    private void stopMotors() {
        for (DcMotor motor : drive) zero(motor);
        for (DcMotor motor : mechanisms) zero(motor);
    }

    private static void zero(DcMotor motor) {
        if (motor != null) {
            try { motor.setPower(0); }
            catch (RuntimeException ex) { RobotLog.ee("PollenCameraDrive", ex, "Could not zero a motor"); }
        }
    }

    @Override public void stop() {
        started = false;
        stopMotors();
        try { if (viewer != null) viewer.close(); }
        finally { if (camera != null) camera.stop(); }
    }
}
