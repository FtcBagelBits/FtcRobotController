package org.firstinspires.ftc.teamcode.pollen;

/** Pure autonomous controller shared by dry-run, drive-only and live robot execution. */
public final class PollenHiveAuto {
    public enum Run { MATCH, SHOT_ONLY, GARDEN_RETURN }
    public enum Phase { DELAY, FIRST, COAST, GARDEN, SETTLE, SECOND, PARK, DONE }
    public static final class Output {
        public double forward,strafe,turn,launcher,intake,agitator;
        public boolean finished,feed;
        public String phase,reason;
    }
    private final PollenHiveRoute config;
    private final Run run;
    private final IsHiveReadyForShot hive;
    private Phase phase=Phase.DELAY;
    private PollenHiveMotion motion;
    private double began,phaseAt,previous=Double.NaN,feedSeconds,settledAt=Double.NaN,px,py,ph;
    private boolean started,wasFeeding;
    private String reason="",motionPhase="";
    public PollenHiveAuto(PollenHiveRoute config,IsHiveReadyForShot hive) { this(config,hive,Run.MATCH); }
    public PollenHiveAuto(PollenHiveRoute config,IsHiveReadyForShot hive,Run run) {
        config.validate(); this.config=config; this.hive=hive; this.run=run;
    }
    public void start(double now) { began=phaseAt=now; previous=now; started=true; hive.reset(); }
    public Output stop(String why) { phase=Phase.DONE; reason=why; wasFeeding=false; return zero(); }
    private Output zero() { Output o=new Output(); o.phase=phase.toString(); o.reason=reason; o.finished=phase==Phase.DONE; return o; }
    public Output tick(double now,IsHiveReadyForShot.Frame frame,double x,double y,double h,double velocity,boolean odometryReady) {
        if(!started) return stop("Not started");
        if(phase==Phase.DONE) return zero();
        double dt=now-previous; previous=now;
        if(!Double.isFinite(now)||dt<0||dt>.25) return stop("Control loop stalled/invalid clock");
        if(now-began>=27) return stop("27 second cutoff");
        if(!odometryReady||!Double.isFinite(x)||!Double.isFinite(y)||!Double.isFinite(h)||!Double.isFinite(velocity))
            return stop("Invalid odometry/launcher reading");
        if(wasFeeding) feedSeconds+=dt;
        wasFeeding=false;
        boolean atPose=Math.hypot(x,y)<=.5&&Math.abs(PollenOdometryDrive.wrap(h))<=Math.toRadians(2);
        double speed=dt>0?Math.hypot(x-px,y-py)/dt:Double.POSITIVE_INFINITY;
        double omega=dt>0?Math.abs(PollenOdometryDrive.wrap(h-ph))/dt:Double.POSITIVE_INFINITY;
        px=x; py=y; ph=h;
        if(atPose&&speed<=1.5&&omega<=Math.toRadians(10)) { if(Double.isNaN(settledAt)) settledAt=now; }
        else settledAt=Double.NaN;
        boolean settled=Double.isFinite(settledAt)&&now-settledAt>=.3;
        boolean shootingPhase=phase==Phase.FIRST||phase==Phase.SECOND||phase==Phase.SETTLE;
        if(shootingPhase&&settled) hive.update(frame,now); else hive.reset();
        double parkReserve=run==Run.MATCH?PollenHiveRoute.planned(config.park())+1:0;
        double remaining=27-(now-began);
        Output o=zero();
        switch(phase) {
            case DELAY:
                if(now-phaseAt>=config.v[2]) enter(run==Run.GARDEN_RETURN?Phase.COAST:Phase.FIRST,now);
                break;
            case FIRST: case SECOND:
                if(remaining<=parkReserve) { park(now,"Time reserved for departure"); break; }
                if(config.v[0]==1) { // Route-only bypasses shots, never vision-gates movement.
                    if(run==Run.SHOT_ONLY) return stop("Shot test needs DRY or LIVE execution");
                    if(phase==Phase.FIRST&&!config.secondary) enter(Phase.COAST,now); else park(now,"Drive-only departure");
                    break;
                }
                double requested=phase==Phase.FIRST?config.v[3]:config.v[4];
                boolean tipped=hive.state(now)==IsHiveReadyForShot.State.DOWN&&feedSeconds>0;
                boolean firstWindow=phase==Phase.FIRST&&!config.secondary&&now-phaseAt>=config.v[5];
                if(feedSeconds>=requested||tipped||firstWindow) {
                    if(run==Run.SHOT_ONLY) return stop(firstWindow?"Shot window ended":"Stationary shot complete");
                    if(phase==Phase.FIRST&&!config.secondary&&feedSeconds>0) enter(Phase.COAST,now);
                    else park(now,firstWindow?"No first-shot feed before window ended":"Shot complete");
                    break;
                }
                o.launcher=config.v[6];
                // No timer override for speed in these new modes: fresh hive + measured speed required.
                boolean readySpeed=velocity>=config.v[6]*PollenConfig.LAUNCHER_READY_FRACTION;
                o.feed=settled&&hive.isHiveReadyForShot(now)&&readySpeed;
                if(o.feed) { o.intake=config.v[11]; o.agitator=config.v[12]; wasFeeding=true; }
                reason=!settled?"Not settled at shooting pose":!readySpeed?"Waiting for measured launcher speed":hive.reason;
                break;
            case COAST:
                // Intake is also the feeder; do not collect while the flywheel is coasting fast.
                if(Math.abs(velocity)<=100 && now-phaseAt>=.5) {
                    double need=PollenHiveRoute.planned(config.garden())+(run==Run.GARDEN_RETURN?2:3+config.v[4]+parkReserve+1);
                    if(remaining<need) park(now,"Garden skipped: not enough estimated cycle time");
                    else { motion=new PollenHiveMotion(config.garden()); enter(Phase.GARDEN,now); }
                } else if(now-phaseAt>2) park(now,"Garden skipped: flywheel did not stop");
                break;
            case GARDEN: case PARK:
                PollenHiveMotion.Output m=motion.tick(now,x,y,h); motionPhase=m.phase;
                if(m.fault!=null) return stop(m.fault);
                if(m.done) {
                    if(phase==Phase.GARDEN) { enter(Phase.SETTLE,now); hive.reset(); settledAt=Double.NaN; }
                    else return stop("Finished departure");
                } else {
                    o.forward=m.forward; o.strafe=m.strafe; o.turn=m.turn;
                    if(m.collect&&config.v[0]!=1) o.intake=config.v[9];
                }
                break;
            case SETTLE:
                if(remaining<=parkReserve) { park(now,"No time for second shot"); break; }
                if(settled) {
                    if(run==Run.GARDEN_RETURN) return stop("Garden returned and settled at S");
                    enter(Phase.SECOND,now);
                }
                else if(now-phaseAt>1.5) return stop("Return pose did not settle");
                break;
            default: break;
        }
        o.finished=phase==Phase.DONE;
        o.phase=phase+((phase==Phase.GARDEN||phase==Phase.PARK)?": "+motionPhase:""); o.reason=reason;
        return o;
    }
    private void enter(Phase p,double now) { phase=p; phaseAt=now; feedSeconds=0; wasFeeding=false; }
    private void park(double now,String why) {
        if(run!=Run.MATCH) { stop(why); return; }
        reason=why; motion=new PollenHiveMotion(config.park()); enter(Phase.PARK,now); }
}
