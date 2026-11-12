package org.firstinspires.ftc.teamcode.pollen;

/** Empirical classifier for ONE tag on the near moving cell, at ONE shooting position.
 * Camera-space metres: x right, y down, z forward. No field pose or assumed hinge geometry.
 * Call update every loop. Call isHiveReadyForShot with the current monotonic time, never cache it.
 * The 14-inch lens height is reproduced during calibration, not subtracted from these coordinates.
 */
public final class IsHiveReadyForShot {
    public enum State { UP, DOWN, UNKNOWN }
    public static final double MAX_AGE = 0.20, CONFIRM_SECONDS = 0.15;
    public static final int CONFIRM_FRAMES = 4;
    public static final class Frame {
        public final double stamp, x, y, z;
        public final boolean valid;
        public Frame(double stamp, double x, double y, double z, boolean valid) {
            this.stamp=stamp; this.x=x; this.y=y; this.z=z; this.valid=valid;
        }
        public boolean usable() {
            return valid && Double.isFinite(stamp) && stamp>0 && Double.isFinite(x)
                    && Double.isFinite(y) && Double.isFinite(z) && z>0.05 && z<15;
        }
    }
    /** Closed 3-D envelope. Keeping range and horizontal position rejects many wrong viewpoints. */
    public static final class Box {
        public final double[] lo, hi;
        public Box(double[] lo, double[] hi) {
            if (lo.length!=3 || hi.length!=3) throw new IllegalArgumentException("Three coordinates required");
            this.lo=lo.clone(); this.hi=hi.clone();
            for(int i=0;i<3;i++) if(!Double.isFinite(lo[i]) || !Double.isFinite(hi[i]) || lo[i]>hi[i])
                throw new IllegalArgumentException("Invalid calibration bounds");
        }
        public boolean contains(Frame f) {
            double[] v={f.x,f.y,f.z};
            for(int i=0;i<3;i++) if(v[i]<lo[i] || v[i]>hi[i]) return false;
            return true;
        }
        public boolean overlaps(Box b) {
            for(int i=0;i<3;i++) if(hi[i]<b.lo[i] || b.hi[i]<lo[i]) return false;
            return true;
        }
    }
    public static final class Profile {
        public final int pipeline, tag;
        public final Box up, down; // down=null means absence/unrecognized DOWN remains UNKNOWN.
        public Profile(int pipeline, int tag, Box up, Box down) {
            if(pipeline<0 || pipeline>9 || tag<0 || tag>586 || up==null
                    || (down!=null && up.overlaps(down)))
                throw new IllegalArgumentException("Invalid/overlapping UP and DOWN calibration");
            this.pipeline=pipeline; this.tag=tag; this.up=up; this.down=down;
        }
    }
    /** Gather only distinct, usable frames while the real hive is stationary in a labeled state. */
    public static final class Capture {
        private final double[] lo={Double.POSITIVE_INFINITY,Double.POSITIVE_INFINITY,Double.POSITIVE_INFINITY};
        private final double[] hi={Double.NEGATIVE_INFINITY,Double.NEGATIVE_INFINITY,Double.NEGATIVE_INFINITY};
        private double stamp=Double.NaN;
        public int count;
        public final StringBuilder samples=new StringBuilder("timestamp,x_m,y_m,z_m\n");
        public void add(Frame f) {
            if(f==null || !f.usable() || f.stamp==stamp) return;
            stamp=f.stamp;
            double[] v={f.x,f.y,f.z};
            for(int i=0;i<3;i++) { lo[i]=Math.min(lo[i],v[i]); hi[i]=Math.max(hi[i],v[i]); }
            count++;
            samples.append(f.stamp).append(",").append(f.x).append(",").append(f.y).append(",").append(f.z).append("\n");
        }
        public Box finish() {
            if(count<40) throw new IllegalArgumentException("Need 40 distinct valid frames; got "+count);
            double[] a=lo.clone(), b=hi.clone();
            // Intentionally narrow initial margins; do not widen to hide poor observations.
            double[] margin={0.025,0.025,0.04};
            for(int i=0;i<3;i++) {
                if(hi[i]-lo[i]>(i==2?0.20:0.12))
                    throw new IllegalArgumentException("Pose too noisy/moving: repeat stationary capture");
                a[i]-=margin[i]; b[i]+=margin[i];
            }
            return new Box(a,b);
        }
    }
    private final Profile profile;
    private double lastStamp=Double.NaN, lastFresh=Double.NEGATIVE_INFINITY, since;
    private State state=State.UNKNOWN, candidate=State.UNKNOWN;
    private int count;
    public String reason="No observation";
    public IsHiveReadyForShot(Profile profile) { this.profile=profile; }
    public void reset() { state=candidate=State.UNKNOWN; count=0; lastFresh=Double.NEGATIVE_INFINITY; }
    public void update(Frame f, double now) {
        if(!Double.isFinite(now)) { reset(); reason="Invalid clock"; return; }
        if(now-lastFresh>MAX_AGE || now<lastFresh) reset();
        if(f==null || !f.usable() || profile==null) { reset(); reason="No usable selected-tag pose/profile"; return; }
        if(f.stamp==lastStamp) { reason="Waiting for new camera frame"; return; }
        if(Double.isFinite(lastStamp) && f.stamp<lastStamp) { reset(); }
        lastStamp=f.stamp; lastFresh=now;
        State observed=profile.up.contains(f)?State.UP:
                profile.down!=null && profile.down.contains(f)?State.DOWN:State.UNKNOWN;
        // An out-of-envelope or opposite frame revokes permission immediately.
        if(observed!=state) state=State.UNKNOWN;
        if(observed==State.UNKNOWN) { candidate=State.UNKNOWN; count=0; reason="Outside calibrated envelopes"; return; }
        if(observed!=candidate) { candidate=observed; count=1; since=now; } else count++;
        if(count>=CONFIRM_FRAMES && now-since>=CONFIRM_SECONDS) state=observed;
        reason=state==State.UNKNOWN?"Confirming "+observed:"Stable "+state;
    }
    public State state(double now) {
        return Double.isFinite(now) && now>=lastFresh && now-lastFresh<=MAX_AGE?state:State.UNKNOWN;
    }
    public boolean isHiveReadyForShot(double now) { return state(now)==State.UP; }
}
