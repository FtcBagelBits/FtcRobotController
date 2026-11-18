package org.firstinspires.ftc.teamcode.pollen;

import java.util.Locale;

/** Pure autonomous controller shared by dry-run, drive-only and live robot execution. */
public final class PollenHiveAuto {
    private static final double HIVE_UP_BEFORE_VOLLEY_S = 1.0;
    private static final double MAX_LOOP_GAP_S = 0.25;
    public enum Run { MATCH, SHOT_ONLY, GARDEN_RETURN }
    /** ROOM = make room: driving to the corner; WAIT = parked in the corner until park time. */
    public enum Phase { DELAY, FIRST, GARDEN, SETTLE, SECOND, ROOM, WAIT, PARK, DONE }
    public static final class Output {
        public double forward,strafe,turn,launcher,intake,agitator;
        public boolean finished,feed;
        public String phase,reason,decision;
    }
    private final PollenHiveRoute config;
    private final Run run;
    private final IsHiveReadyForShot hive;
    private Phase phase=Phase.DELAY;
    private PollenHiveMotion motion;
    private double began,phaseAt,previous=Double.NaN,feedSeconds,settledAt=Double.NaN,px,py,ph;
    private double hiveUpSince=Double.NaN;
    private boolean started,wasFeeding,departed,hiveUpLatched;
    private String reason="",motionPhase="",decision="before first tick";
    public PollenHiveAuto(PollenHiveRoute config,IsHiveReadyForShot hive) { this(config,hive,Run.MATCH); }
    public PollenHiveAuto(PollenHiveRoute config,IsHiveReadyForShot hive,Run run) {
        config.validate(); this.config=config; this.hive=hive; this.run=run;
    }
    public void start(double now) { began=phaseAt=now; previous=now; started=true; hive.reset(); }
    public Output stop(String why) { phase=Phase.DONE; reason=why; wasFeeding=false; return zero(); }
    private Output zero() { Output o=new Output(); o.phase=phase.toString(); o.reason=reason; o.decision=decision; o.finished=phase==Phase.DONE; return o; }
    public Output tick(double now,IsHiveReadyForShot.Frame frame,double x,double y,double h,double velocity,boolean odometryReady) {
        if(!started) return stop("Not started");
        if(phase==Phase.DONE) return zero();
        double dt=now-previous; previous=now;
        decision=String.format(Locale.US,"evaluated=%s dt=%.4f odometry_ready=%s launcher_tps=%.2f",phase,dt,odometryReady,velocity);
        if(!Double.isFinite(now)||dt<0||dt>MAX_LOOP_GAP_S) return stop("Control loop stalled/invalid clock");
        if(now-began>=27) return stop("27 second cutoff");
        if(!odometryReady||!Double.isFinite(x)||!Double.isFinite(y)||!Double.isFinite(h)||!Double.isFinite(velocity))
            return stop("Invalid odometry/launcher reading");
        boolean volleyStarted=wasFeeding;
        if(volleyStarted) feedSeconds+=dt;
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
        boolean hiveReady=hive.isHiveReadyForShot(now);
        if(phase==Phase.FIRST||phase==Phase.SECOND) {
            if(!hiveReady) hiveUpSince=Double.NaN;
            else if(Double.isNaN(hiveUpSince)) hiveUpSince=now;
            if(now-hiveUpSince>HIVE_UP_BEFORE_VOLLEY_S) hiveUpLatched=true;
        }
        boolean readySpeed=velocity>=config.v[6]*PollenConfig.LAUNCHER_READY_FRACTION;
        double parkReserve=departureReserve();
        double remaining=27-(now-began);
        double gardenNeed=PollenHiveRoute.planned(config.garden())+(run==Run.GARDEN_RETURN?2:3+config.v[4]+parkReserve+1);
        // This describes the decision INPUTS, before any transition resets phase/feed timers.
        decision=String.format(Locale.US,
                "evaluated=%s dt=%.4f phase_s=%.3f remaining_s=%.3f feed_s=%.3f requested_s=%.3f park_reserve_s=%.3f garden_need_s=%.3f at_pose=%s settled=%s speed_in_s=%.2f omega_deg_s=%.2f speed_ready=%s hive_ready=%s hive_up_s=%.3f hive_up_latched=%s",
                phase,dt,now-phaseAt,remaining,feedSeconds,phase==Phase.SECOND?config.v[4]:config.v[3],parkReserve,gardenNeed,
                atPose,settled,speed,Math.toDegrees(omega),readySpeed,hiveReady,
                hiveReady&&!Double.isNaN(hiveUpSince)?now-hiveUpSince:0,hiveUpLatched);
        // A shot/delay should never move the chassis. Stop, rather than starting a route
        // from a suspect pose. This cannot distinguish physical movement from an odometry fault.
        if((!departed||volleyStarted) && (Math.hypot(x,y)>2 || Math.abs(PollenOdometryDrive.wrap(h))>Math.toRadians(10)))
            return stop("Unexpected motion at shooting start (>2 in or >10 deg); inspect drive/mapping/odometry");
        Output o=zero();
        switch(phase) {
            case DELAY:
                if(now-phaseAt>=config.v[2]) {
                    if(run==Run.GARDEN_RETURN) garden(now,"Garden practice"); else enter(Phase.FIRST,now);
                }
                break;
            case FIRST: case SECOND:
                if(remaining<=parkReserve) { park(now,"Time reserved for departure"); break; }
                if(config.v[0]==1) { // Route-only bypasses shots, never vision-gates movement.
                    if(run==Run.SHOT_ONLY) return stop("Shot test needs DRY or LIVE execution");
                    if(phase==Phase.FIRST&&!config.secondary) garden(now,"Drive-only skips shot"); else park(now,"Drive-only departure");
                    break;
                }
                double requested=phase==Phase.FIRST?config.v[3]:config.v[4];
                boolean firstWindow=!volleyStarted&&phase==Phase.FIRST&&!config.secondary&&now-phaseAt>=config.v[5];
                if(feedSeconds>=requested||firstWindow) {
                    if(run==Run.SHOT_ONLY) return stop(firstWindow?"Shot window ended":"Stationary shot complete");
                    if(phase==Phase.FIRST&&!config.secondary&&feedSeconds>0) {
                        garden(now,"First feed duration complete");
                    }
                    else park(now,firstWindow?"No first-shot feed before window ended":"Shot complete");
                    break;
                }
                if(!volleyStarted&&remaining<=parkReserve+requested+MAX_LOOP_GAP_S) {
                    park(now,"Insufficient time for full volley before departure");
                    break;
                }
                o.launcher=config.v[6];
                // After feed starts, finish the volley through camera changes and flywheel dips.
                o.feed=volleyStarted||(settled&&hiveUpLatched&&readySpeed);
                if(o.feed) { o.intake=config.v[11]; o.agitator=config.v[12]; wasFeeding=true; }
                reason=o.feed?"Feeding: hive UP latched":!settled?"Not settled at shooting pose":
                        !readySpeed?"Waiting for measured launcher speed":hiveReady?"Timing confirmed hive UP":hive.reason;
                break;
            case GARDEN: case ROOM: case PARK:
                PollenHiveMotion.Output m=motion.tick(now,x,y,h); motionPhase=m.phase; decision+=" "+m.detail;
                if(m.fault!=null) return stop(m.fault);
                if(m.done) {
                    if(phase==Phase.GARDEN) { enter(Phase.SETTLE,now); hive.reset(); settledAt=Double.NaN; }
                    else if(phase==Phase.ROOM) {
                        enter(Phase.WAIT,now);
                        reason=String.format(Locale.US,"In the corner; park with %.1f s left",config.v[PollenHiveRoute.ROOM_PARK_SECONDS_LEFT]);
                        // Late arrival: park on this same tick rather than one loop later.
                        if(remaining<=config.v[PollenHiveRoute.ROOM_PARK_SECONDS_LEFT]) park(now,"Reached the corner late: park now");
                    }
                    else return stop("Finished departure");
                } else {
                    o.forward=m.forward; o.strafe=m.strafe; o.turn=m.turn;
                    if(m.collect&&config.v[0]!=1) o.intake=config.v[9];
                }
                break;
            case WAIT:
                // Sit still in the corner, out of the hero's way, until park time.
                if(remaining<=config.v[PollenHiveRoute.ROOM_PARK_SECONDS_LEFT])
                    park(now,String.format(Locale.US,"%.1f s left: park",remaining));
                break;
            case SETTLE:
                if(remaining<=parkReserve) { park(now,"No time for second shot"); break; }
                if(settled) {
                    if(run==Run.GARDEN_RETURN) return stop("Garden returned and settled at S");
                    enter(Phase.SECOND,now);
                }
                // Off the shot pose: skip the second shot but still park (park() stops in practice runs).
                else if(now-phaseAt>1.5) park(now,"Return pose did not settle; second shot skipped");
                break;
            default: break;
        }
        o.finished=phase==Phase.DONE;
        o.phase=phase+((phase==Phase.GARDEN||phase==Phase.ROOM||phase==Phase.PARK)?": "+motionPhase:""); o.reason=reason; o.decision=decision;
        return o;
    }
    /** The agitator meters balls into the flywheel. Stop both shooting outputs on the
     * transition tick, then drive/collect without waiting for flywheel spin-down. */
    private void garden(double now,String why) {
        double parkReserve=departureReserve();
        double need=PollenHiveRoute.planned(config.garden())+(run==Run.GARDEN_RETURN?2:3+config.v[4]+parkReserve+1);
        if(27-(now-began)<need) { park(now,"Garden skipped: not enough estimated cycle time"); return; }
        reason=why+"; garden cycle budget passed";
        motion=new PollenHiveMotion(config.garden()); enter(Phase.GARDEN,now);
    }
    private void enter(Phase p,double now) {
        phase=p;
        if(p==Phase.GARDEN||p==Phase.ROOM||p==Phase.WAIT||p==Phase.PARK) departed=true;
        phaseAt=now;
        feedSeconds=0;
        wasFeeding=false;
        hiveUpSince=Double.NaN;
        hiveUpLatched=false;
    }
    /** Seconds kept free before 27 s for leaving the shooting spot: the park route, or for make
     * room the corner route plus the late-park time. Practice runs never leave, so 0. */
    private double departureReserve() {
        if(run!=Run.MATCH) return 0;
        if(config.makeRoom) return PollenHiveRoute.planned(config.corner())+config.v[PollenHiveRoute.ROOM_PARK_SECONDS_LEFT]+1;
        return PollenHiveRoute.planned(config.park())+1;
    }
    private void park(double now,String why) {
        if(run!=Run.MATCH) { stop(why); return; }
        if(config.makeRoom&&phase!=Phase.WAIT) {
            // Make room: get out of the hero's way first; WAIT parks with the set seconds left.
            reason=why+"; making room in the corner"; motion=new PollenHiveMotion(config.corner()); enter(Phase.ROOM,now); return;
        }
        reason=why; motion=new PollenHiveMotion(config.makeRoom?config.cornerPark():config.park()); enter(Phase.PARK,now); }
}
