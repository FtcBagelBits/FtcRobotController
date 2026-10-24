import org.firstinspires.ftc.teamcode.pollen.PollenTeleOpTuning;
import static org.firstinspires.ftc.teamcode.pollen.PollenTeleOpTuning.Row.*;

/** PollenBot TeleOp INIT menu: defaults, edits, limits, memory and locking. No robot required. */
public final class PollenTeleOpTuningTest {
    private static int checks;
    private static void check(boolean ok, String message) {
        checks++;
        if (!ok) throw new AssertionError(message);
    }
    private static void near(double actual, double expected, String message) {
        check(Math.abs(actual - expected) < 1e-9, message + ": " + actual + " != " + expected);
    }
    private static void release(PollenTeleOpTuning t) { t.update(false, false, false, false, false); }
    private static void tap(PollenTeleOpTuning t, int direction, int count) {
        for (int i = 0; i < count; i++) { t.update(false, false, direction < 0, direction > 0, false); release(t); }
    }
    private static void select(PollenTeleOpTuning t, PollenTeleOpTuning.Row row) {
        while (t.selected() != row) { t.update(false, true, false, false, false); release(t); }
    }

    public static void main(String[] args) {
        PollenTeleOpTuning t = new PollenTeleOpTuning(1250, 1200, null);
        near(t.launcherVelocity(), 1250, "default launcher target is the TeleOp constant");
        near(t.launcherMinVelocity(), 1200, "default feed gate is the TeleOp constant");
        near(t.driveScale(), 1, "default drive power leaves sticks unchanged");
        check(t.isDefault() && t.selected() == LAUNCHER, "starts on launcher row with defaults");

        tap(t, 1, 3); near(t.launcherVelocity(), 1400, "launcher +50 per tap");
        near(t.launcherMinVelocity(), 1400 * .96, "feed gate keeps the 96% ratio");
        t.update(false, false, false, true, false); t.update(false, false, false, true, false);
        near(t.launcherVelocity(), 1450, "held button does not repeat"); release(t);
        t.update(false, false, true, true, false); near(t.launcherVelocity(), 1450, "opposite arrows cancel"); release(t);
        tap(t, 1, 100); near(t.launcherVelocity(), 2500, "launcher upper limit");
        tap(t, -1, 100); near(t.launcherVelocity(), 500, "launcher lower limit");

        select(t, DRIVE); near(t.launcherVelocity(), 500, "changing rows does not edit");
        tap(t, 1, 1); near(t.driveScale(), 1, "drive cannot exceed 100%");
        tap(t, -1, 4); near(t.driveScale(), .8, "drive -5% per tap");
        tap(t, -1, 100); near(t.driveScale(), .2, "drive lower limit");
        t.update(true, false, false, false, false); release(t);
        check(t.selected() == LAUNCHER, "UP wraps between the two rows");
        check(!t.isDefault(), "changed settings reported");

        PollenTeleOpTuning kept = new PollenTeleOpTuning(1250, 1200, t.snapshot());
        near(kept.launcherVelocity(), 500, "launcher kept for next INIT");
        near(kept.driveScale(), .2, "drive kept for next INIT");
        tap(t, 1, 1); near(kept.launcherVelocity(), 500, "snapshots do not alias");

        t.update(false, false, false, false, true); release(t);
        check(t.isDefault(), "Triangle restores both defaults");
        near(t.launcherVelocity(), 1250, "reset launcher"); near(t.driveScale(), 1, "reset drive");
        t.update(false, false, false, false, true); t.update(false, false, true, false, true);
        near(t.launcherVelocity(), 1250, "reset dominates and does not repeat");
        release(t);

        tap(t, 1, 2); t.lock(); tap(t, 1, 5); t.update(false, false, false, false, true);
        near(t.launcherVelocity(), 1350, "START locks edits and reset");
        check(t.locked(), "locked flag");

        boolean rejected = false;
        try { new PollenTeleOpTuning(1250, 1300, null); } catch (IllegalArgumentException expected) { rejected = true; }
        check(rejected, "feed gate above target rejected");
        System.out.println("PollenTeleOpTuningTest: " + checks + " checks passed");
    }
}
