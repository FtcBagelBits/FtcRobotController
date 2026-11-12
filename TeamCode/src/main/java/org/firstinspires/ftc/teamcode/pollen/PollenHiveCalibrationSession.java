package org.firstinspires.ftc.teamcode.pollen;

/** Candidate lifecycle, independently testable without a Hub. No inferred DOWN-height edits. */
public final class PollenHiveCalibrationSession {
    public IsHiveReadyForShot.Mount saved, live;
    public boolean candidate, upChecked, downChecked;
    public int version, cluster = -1;
    public PollenHiveCalibrationSession(IsHiveReadyForShot.Mount saved) { this.saved = this.live = saved; }
    public void resetChecks() { upChecked = downChecked = false; }
    public void beginTilt() { version++; live = saved; candidate = false; cluster = -1; resetChecks(); }
    public boolean accept(PollenHiveCalibration c) {
        boolean ok = c.problem().isEmpty();
        if (c.tilt) {
            if (ok) { live = c.candidateMount(); cluster = c.cluster; candidate = true; }
        } else {
            // A failed repeat must revoke that state's previous success.
            if (cluster != c.cluster) { resetChecks(); cluster = c.cluster; }
            if (c.expected == IsHiveReadyForShot.State.UP) upChecked = ok;
            else downChecked = ok;
        }
        return ok;
    }
    public boolean canSave() { return live.problem().isEmpty() && upChecked && downChecked; }
    public void saved() { saved = live; candidate = false; }
}
