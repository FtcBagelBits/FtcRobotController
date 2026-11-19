package org.firstinspires.ftc.teamcode.pollen;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.io.IOException;

/** Driver Hub adapter. Match autos can use PollenCollectionSequence without this menu or hardware setup. */
public abstract class PollenCollectionPractice extends OpMode {
    protected abstract PollenCollectionSequence.Kind kind();
    private boolean flower() { return kind() == PollenCollectionSequence.Kind.FLOWER; }
    private String profile() { return flower() ? "flower-practice" : "garden-practice"; }

    private final ElapsedTime clock = new ElapsedTime();
    private final DcMotor[] drive = new DcMotor[4];
    private DcMotor intake;
    private Servo drainer;
    private PinpointLocalizer localizer;
    private GoBildaPinpointDriver pinpoint;
    private PollenHiveSettings.Values settings;
    private PollenCollectionSequence sequence;
    private PollenHiveTrace trace;
    private String fault, message, lastPhase, recording;
    private int row;
    private boolean finished;
    private double elapsed, lastSample, forwardCm, leftCm;
    private double[] entry;

    @Override public void init() {
        row = 0;
        finished = false;
        elapsed = lastSample = forwardCm = leftCm = 0;
        fault = lastPhase = null;
        sequence = null;
        trace = null;
        recording = "";
        settings = PollenCollectionSequence.settings(kind());
        message = PollenHiveStore.load(profile(), settings);
        try {
            drainer = flower() ? hardwareMap.get(Servo.class, PollenDrainerHold.SERVO_NAME)
                    : hardwareMap.tryGet(Servo.class, PollenDrainerHold.SERVO_NAME);
            if (drainer != null) drainer.setPosition(PollenTeleOpTuning.DEFAULT_DRAINER_UP);
            for (int i = 0; i < drive.length; i++) {
                drive[i] = hardwareMap.get(DcMotor.class, PollenConfig.DRIVE_NAMES[i]);
                configure(drive[i], PollenConfig.DRIVE_REVERSED[i]);
            }
            intake = hardwareMap.get(DcMotor.class, "intake");
            configure(intake, false);
            PollenHiveHardware.requireDistinct(new String[]{"frontLeft","frontRight","backLeft","backRight","intake"},
                    new DcMotor[]{drive[0],drive[1],drive[2],drive[3],intake});
            localizer = PollenOdometryHardware.open(hardwareMap);
            pinpoint = PollenOdometryHardware.device(hardwareMap);
        } catch (RuntimeException ex) { block(ex.toString()); }
    }

    private static void configure(DcMotor m, boolean reversed) {
        m.setPower(0);
        m.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override public void init_loop() {
        int n = settings.rows.length;
        if (gamepad1.dpadUpWasPressed()) row = (row + n - 1) % n;
        if (gamepad1.dpadDownWasPressed()) row = (row + 1) % n;
        if (gamepad1.dpadLeftWasPressed()) settings.edit(row, -1);
        if (gamepad1.dpadRightWasPressed()) settings.edit(row, 1);
        if (gamepad1.triangleWasPressed()) settings.reset(row);
        if (gamepad1.circleWasPressed()) settings.resetAll();
        String validation = "Valid";
        try {
            PollenCollectionSequence.validate(kind(), settings);
            if (gamepad1.squareWasPressed()) {
                PollenHiveStore.save(profile(), settings);
                message = "Saved on Hub";
            }
        } catch (IOException | IllegalArgumentException ex) { validation = ex.getMessage(); }
        try { if (localizer != null) localizer.update(); }
        catch (RuntimeException ex) { block(ex.toString()); }
        telemetry.addData("Build / practice", "%s / %s", PollenConfig.BUILD, kind());
        if (flower()) {
            telemetry.addData("Placement", "Drainer centered on flower, %.2f cm back from seated mark", settings.get("gap"));
        } else {
            telemetry.addLine("Intake faces garden wall. Side wall on robot right.");
        }
        telemetry.addData("Pinpoint", PollenOdometryHardware.status(pinpoint));
        telemetry.addData("Settings", fault == null ? validation : fault);
        for (int i = Math.max(0,row - 2); i <= Math.min(n - 1,row + 2); i++)
            telemetry.addLine((i == row ? "> " : "  ") + settings.line(i));
        telemetry.addLine("D-pad up/down: row, left/right: edit. Triangle: reset row. Circle: reset all. Square: save.");
        telemetry.addData("Profile", "%s | %d unsaved edits", message, settings.unsavedCount());
        telemetry.update();
    }

    @Override public void start() {
        clock.reset();
        if (fault != null) return;
        try {
            localizer.update();
            if (!"READY".equals(PollenOdometryHardware.status(pinpoint))) throw new IllegalStateException("Pinpoint must be READY");
            entry = pose();
            sequence = new PollenCollectionSequence(kind(), settings, entry[0], entry[1], entry[2]);
            try {
                trace = PollenRunLog.open(PollenHiveStore.folder(), PollenRunLog.Kind.AUTO, System.currentTimeMillis());
                trace.row(0, true, "START", PollenConfig.BUILD, profile(), settings.overrides());
                trace.row(0, false, "seconds", "phase", "route_x_in", "route_y_in", "heading_deg", "intake", "drainer_down", "event");
                recording = trace.fileName;
            } catch (IOException | RuntimeException ex) { recording = "Recording unavailable: " + ex.getMessage(); }
            step();
        } catch (RuntimeException ex) { block(ex.toString()); }
    }

    private double[] pose() {
        Pose p = localizer.pose();
        return PollenFacing.routePose(p.x(), p.y(), p.heading());
    }

    @Override public void loop() {
        try { if (fault == null && !finished) step(); }
        catch (RuntimeException ex) { block(ex.toString()); }
        telemetry.addData("Phase", fault == null ? lastPhase : "BLOCKED: " + fault);
        telemetry.addData("Elapsed", "%.2f s", elapsed);
        telemetry.addData("From START", "%+.1f cm forward / %+.1f cm left", forwardCm, leftCm);
        telemetry.addData("Recording", trace == null ? recording : recording + " / " + trace.status);
        telemetry.update();
    }

    private void step() {
        localizer.update();
        double[] p = pose();
        double now = clock.seconds();
        elapsed = now;
        PollenCollectionSequence.Output o = sequence.tick(now, p[0], p[1], p[2],
                "READY".equals(PollenOdometryHardware.status(pinpoint)));
        if (o.fault != null) { block(o.fault); return; }
        double[] delta = PollenFacing.routePose(p[0] - entry[0], p[1] - entry[1], 0);
        double cos = Math.cos(entry[2]), sin = Math.sin(entry[2]);
        forwardCm = (cos * delta[0] + sin * delta[1]) * 2.54;
        leftCm = (-sin * delta[0] + cos * delta[1]) * 2.54;
        intake.setPower(o.intake);
        if (drainer != null) drainer.setPosition(o.drainerDown ? settings.get("down") : PollenTeleOpTuning.DEFAULT_DRAINER_UP);
        double[] powers = PollenFacing.wheelPowers(o.forward, o.strafe, o.turn);
        for (int i = 0; i < drive.length; i++) drive[i].setPower(powers[i]);
        if (trace != null && (!o.phase.equals(lastPhase) || now - lastSample >= .1 || o.contactLimit != null)) {
            trace.row(now, false, now, o.phase, p[0], p[1], Math.toDegrees(p[2]), o.intake, o.drainerDown,
                    o.contactLimit == null ? "" : "CONTACT LIMIT: " + o.contactLimit);
            lastSample = now;
        }
        lastPhase = o.phase;
        finished = o.done;
    }

    private void block(String why) {
        fault = why;
        zero();
        if (trace != null) trace.row(clock.seconds(), true, clock.seconds(), "FAULT", lastPhase, why);
    }

    private void zero() {
        for (DcMotor m : drive) if (m != null) try { m.setPower(0); } catch (RuntimeException ignored) { }
        if (drainer != null) try { drainer.setPosition(PollenTeleOpTuning.DEFAULT_DRAINER_UP); } catch (RuntimeException ignored) { }
        if (intake != null) try { intake.setPower(0); } catch (RuntimeException ignored) { }
    }

    @Override public void stop() {
        if (sequence != null) sequence.stop();
        zero();
        if (trace != null) {
            trace.row(clock.seconds(), true, clock.seconds(), "STOP", lastPhase);
            trace.close();
        }
    }
}
