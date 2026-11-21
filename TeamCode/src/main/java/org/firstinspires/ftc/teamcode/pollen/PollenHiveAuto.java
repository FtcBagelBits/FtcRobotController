package org.firstinspires.ftc.teamcode.pollen;

import java.util.Arrays;
import java.util.Locale;

/** Pure autonomous controller shared by dry-run, drive-only and live robot execution. */
public final class PollenHiveAuto {
    private static final double HIVE_UP_BEFORE_VOLLEY_S = 1.0;
    private static final double RETURN_ALIGNMENT_DEG = .75;
    /** Beta/match return volley: how far from the shot spot still counts as there (inches).
     * 2026-11-21 v48-v50 primary angled returns stopped 0.40-0.65 in off, and the two past 0.5 in
     * skipped a loaded volley; 1 in keeps those shots. The first shot keeps 0.5 in. */
    static final double RETURN_SHOT_TOLERANCE_IN = 1.0;
    /** Beta/match modes always shoot once they reach the spot (v53): too many balls on board risks
     * a penalty. Within this distance the robot still turns to the aim first; further off it shoots
     * from where it stopped. Camera rules are unchanged: the primary's first volley never checks
     * (its cell starts UP); its return volley and every secondary volley need the cell UP. */
    static final double MUST_SHOOT_ALIGN_IN = 3.0;
    private static final double MAX_LOOP_GAP_S = 0.25;
    public enum Run { MATCH, SHOT_ONLY, GARDEN_RETURN }
    /** ROOM = make room: driving to the corner; WAIT = parked in the corner until park time. */
    public enum Phase { DELAY, TO_SHOT, FIRST, OUTBOUND, GARDEN, FLOWER, RETURN, SETTLE, ALIGN_SHOT, SECOND, ROOM, WAIT, PARK_WAIT, PARK, DONE }
    public static final class Output {
        public double forward,strafe,turn,launcher,intake,agitator;
        public boolean finished,feed,drainerDown;
        public String phase,reason,decision;
    }
    private final PollenHiveRoute config;
    private final Run run;
    private final IsHiveReadyForShot hive;
    private final PollenCollectionSequence.Kind collectionKind;
    private final double collectionEstimate, collectionParkReserve;
    private final int cutoffSeconds;
    /** Shooting pose: the start, or the angled mode's spot and aim heading (inches, radians). */
    private final double shotX,shotY,shotH;
    /** The first volley is over (or skipped); before that, angled SETTLE leads to FIRST. */
    private boolean firstShotDone;
    private Phase phase=Phase.DELAY;
    private PollenHiveMotion motion;
    private PollenCollectionSequence collection;
    private double began,phaseAt,previous=Double.NaN,feedSeconds,settledAt=Double.NaN,px,py,ph;
    private double hiveUpSince=Double.NaN;
    private double alignUntil;
    /** Clear-out after a volley: agitator and launcher keep running until this time. */
    private double clearOutUntil=Double.NEGATIVE_INFINITY;
    private boolean started,wasFeeding,departed,hiveUpLatched;
    /** Paced feed: agitator held while the launcher recovers. Fast mode: return nudge used. */
    private boolean feedPaused,nudged;
    /** Primary: the aim turn gave up for this approach, so shoot from the current heading. */
    private boolean aimGivenUp;
    /** Where the current volley started: the motion check measures from here (v53). */
    private double volleyX,volleyY,volleyH;
    private String reason="",motionPhase="",decision="before first tick";
    public PollenHiveAuto(PollenHiveRoute config,IsHiveReadyForShot hive) { this(config,hive,Run.MATCH); }
    public PollenHiveAuto(PollenHiveRoute config,IsHiveReadyForShot hive,Run run) {
        this.config=config; this.hive=hive; this.run=run;
        collectionKind=run==Run.MATCH&&!config.makeRoom?config.collectionKind():null;
        config.validateForStart(collectionKind!=null);
        collectionEstimate=collectionKind==null?0:config.collectionEstimate();
        collectionParkReserve=collectionKind==null?0:config.collectionParkSeconds()+config.parkMargin()+config.aimTurnSeconds();
        cutoffSeconds=collectionKind==null?27:30;
        shotX=config.shotX(); shotY=config.shotY(); shotH=config.shotHeading();
    }
    /** Angled mode drives to its shot spot before the first volley (match runs only). */
    private boolean angledStart() { return config.angled&&collectionKind!=null; }
    /** Beta/match modes never skip a volley for being off the spot (v53): too many balls on board
     * risks a penalty. The secondary still waits for its cell to tip UP before the first volley. */
    private boolean mustShoot() { return collectionKind!=null; }
    public void start(double now) { start(now,now); }
    void start(double now,double timerStart) { began=timerStart; phaseAt=previous=now; started=true; hive.reset(); }
    /** Which volleys wait for the camera to latch our cell UP. The secondary's preload never needs
     * the partner again: its cell needs about 8 balls to tip, so after our 4 preload pollen it is
     * still UP when we come back from the flower. Its return volley (Secondary Shooter + Flower)
     * feeds as soon as it is settled with the launcher at speed (v40, match 2). The primary's
     * return volley still waits for UP: its preload tips its own cell DOWN. */
    boolean checksHive() {
        if(phase==Phase.SECOND) return !(config.secondary&&collectionKind!=null);
        return phase==Phase.FIRST&&config.secondary;
    }
    public Output stop(String why) { phase=Phase.DONE; reason=why; wasFeeding=false; endClearOut(); return zero(); }
    private Output zero() { Output o=new Output(); o.phase=phase.toString(); o.reason=reason; o.decision=decision; o.finished=phase==Phase.DONE; return o; }
    public Output tick(double now,IsHiveReadyForShot.Frame frame,double x,double y,double h,double velocity,boolean odometryReady) {
        if(!started) return stop("Not started");
        if(phase==Phase.DONE) return zero();
        double dt=now-previous; previous=now;
        decision=String.format(Locale.US,"evaluated=%s dt=%.4f odometry_ready=%s launcher_tps=%.2f",phase,dt,odometryReady,velocity);
        if(!Double.isFinite(now)||dt<0||dt>MAX_LOOP_GAP_S) return stop("Control loop stalled/invalid clock");
        if(now-began>=cutoffSeconds) return stop(collectionKind==null?"27 second cutoff":"30 second cutoff during "+phase
                +(motionPhase.isEmpty()?"":": "+motionPhase));
        if(!odometryReady||!Double.isFinite(x)||!Double.isFinite(y)||!Double.isFinite(h)||!Double.isFinite(velocity))
            return stop("Invalid odometry/launcher reading");
        boolean volleyStarted=wasFeeding;
        if(volleyStarted) feedSeconds+=dt;
        wasFeeding=false;
        double distanceFromShot=Math.hypot(x-shotX,y-shotY);
        double headingError=Math.toDegrees(PollenOdometryDrive.wrap(h-shotH));
        boolean atShotPosition=distanceFromShot<=(collectionKind!=null&&firstShotDone
                ?RETURN_SHOT_TOLERANCE_IN:PollenOdometryDrive.POSITION_TOLERANCE_IN);
        boolean headingNeedsCorrection=Math.abs(headingError)>PollenOdometryDrive.HEADING_TOLERANCE_DEG;
        boolean atPose=atShotPosition&&!headingNeedsCorrection;
        double speed=dt>0?Math.hypot(x-px,y-py)/dt:Double.POSITIVE_INFINITY;
        double turnRate=dt>0?PollenOdometryDrive.wrap(h-ph)/dt:Double.POSITIVE_INFINITY;
        double omega=Math.abs(turnRate);
        px=x; py=y; ph=h;
        // Keep the shooting motion check ahead of any deadline-driven departure.
        // Angled: before it leaves for its spot, the robot should still be at the start.
        boolean atStartCheck=!departed&&angledStart();
        // During a volley, measure from where it started: a primary may shoot from off the spot.
        double fromCheck=atStartCheck?Math.hypot(x,y):volleyStarted?Math.hypot(x-volleyX,y-volleyY):distanceFromShot;
        double headingCheck=atStartCheck?Math.toDegrees(PollenOdometryDrive.wrap(h))
                :volleyStarted?Math.toDegrees(PollenOdometryDrive.wrap(h-volleyH)):headingError;
        if((!departed||volleyStarted) && (fromCheck>2 || Math.abs(headingCheck)>10))
            return stop("Unexpected motion at shooting start (>2 in or >10 deg); inspect drive/mapping/odometry");
        double remaining=cutoffSeconds-(now-began);
        double parkReserve=departureReserve();
        if(collectionKind!=null&&config.parkEnabled()&&phase!=Phase.PARK) {
            // Already returning: finish the legs left, never rebuild the return from the collection site.
            // A robot that shot from near the spot parks from where it is (v53), not via the spot.
            double nearSpot=mustShoot()&&(phase==Phase.SETTLE||phase==Phase.ALIGN_SHOT||phase==Phase.SECOND
                    ||phase==Phase.PARK_WAIT)?MUST_SHOOT_ALIGN_IN:1.5;
            PollenHiveRoute.Node[] approach=distanceFromShot<=nearSpot?null
                    :phase==Phase.RETURN&&motion!=null?motion.remaining():config.collectionReturn(x,y);
            if(approach!=null&&approach.length==0) approach=null;
            // Angled: the park starts by turning back from the aim heading.
            parkReserve=collectionParkReserve;
            if(approach!=null) parkReserve+=PollenHiveRoute.planned(approach,x,y);
            if(remaining<=parkReserve) {
                startBetaPark(now,String.format(Locale.US,"Park deadline during %s, %.2f s left",phase,remaining),approach);
                volleyStarted=false;
            }
        }
        // Beta at the spot: stopped is enough; it shoots from wherever the approach ended.
        // Fast mode first tries its one nudge back onto the spot.
        boolean poseOk=atPose||(mustShoot()&&departed&&(!config.fast||nudged)
                &&(phase==Phase.SETTLE||phase==Phase.FIRST||phase==Phase.SECOND));
        if(poseOk&&speed<=1.5&&omega<=Math.toRadians(10)) { if(Double.isNaN(settledAt)) settledAt=now; }
        else settledAt=Double.NaN;
        boolean settled=Double.isFinite(settledAt)&&now-settledAt>=.3;
        boolean checksHive=checksHive();
        if(checksHive&&settled) hive.updateForAutonomous(frame,now); else hive.reset();
        boolean hiveReady=checksHive&&hive.isHiveReadyForShot(now);
        if(checksHive) {
            if(!hiveReady) {
                hiveUpSince=Double.NaN;
                if(!volleyStarted) hiveUpLatched=false;
            }
            else if(Double.isNaN(hiveUpSince)) hiveUpSince=now;
            if(now-hiveUpSince>HIVE_UP_BEFORE_VOLLEY_S) hiveUpLatched=true;
        }
        boolean readySpeed=velocity>=config.v[6]*PollenConfig.LAUNCHER_READY_FRACTION;
        double requested=config.feedDuration(phase==Phase.SECOND,collectionKind!=null);
        double collectionNeed=collectionKind!=null?collectionEstimate:
                run==Run.GARDEN_RETURN?PollenHiveRoute.planned(config.garden())+2:0;
        // This describes the decision INPUTS, before any transition resets phase/feed timers.
        decision=String.format(Locale.US,
                "evaluated=%s dt=%.4f phase_s=%.3f remaining_s=%.3f feed_s=%.3f requested_s=%.3f park_reserve_s=%.3f collection_need_s=%.3f at_pose=%s settled=%s speed_in_s=%.2f omega_deg_s=%.2f speed_ready=%s hive_checked=%s hive_ready=%s hive_up_s=%.3f hive_up_latched=%s",
                phase,dt,now-phaseAt,remaining,feedSeconds,requested,parkReserve,collectionNeed,
                atPose,settled,speed,Math.toDegrees(omega),readySpeed,checksHive,hiveReady,
                hiveReady&&!Double.isNaN(hiveUpSince)?now-hiveUpSince:0,hiveUpLatched);
        Output o=zero();
        switch(phase) {
            case DELAY:
                if(now-phaseAt>=config.v[2]) {
                    if(run==Run.GARDEN_RETURN) garden(now,"Garden practice");
                    else if(angledStart()) {
                        reason=String.format(Locale.US,"Driving to the angled shot spot (%.1f in away, %.1f in left, aim %.1f deg)",
                                shotX,shotY,Math.toDegrees(shotH));
                        motion=new PollenHiveMotion(config.toShot());
                        enter(Phase.TO_SHOT,now);
                    }
                    else enter(Phase.FIRST,now);
                }
                break;
            case FIRST: case SECOND:
                boolean returnVolley=phase==Phase.SECOND&&collectionKind!=null;
                if((returnVolley||angledStart())&&!volleyStarted&&!aimGivenUp&&atShotPosition&&headingNeedsCorrection) {
                    alignForReturnShot(now,headingError);
                    break;
                }
                if(collectionKind==null&&remaining<=parkReserve) { park(now,"Time reserved for departure"); break; }
                if(config.v[0]==1) { // Route-only bypasses shots, never vision-gates movement.
                    if(run==Run.SHOT_ONLY) return stop("Shot test needs DRY or LIVE execution");
                    if(phase==Phase.FIRST&&collectionKind!=null) outbound(now,"Drive-only skips shot");
                    else if(returnVolley) waitForBetaPark(now,"Returned in DRIVE");
                    else park(now,"Drive-only departure");
                    break;
                }
                boolean firstWindow=collectionKind==null&&!volleyStarted&&phase==Phase.FIRST&&!config.secondary&&now-phaseAt>=config.v[5];
                if(feedSeconds>=requested||firstWindow) {
                    if(run==Run.SHOT_ONLY) return stop(firstWindow?"Shot window ended":"Stationary shot complete");
                    // A finished volley may leave balls behind: keep clearing while the robot moves on.
                    if(collectionKind!=null&&feedSeconds>0) startClearOut(now);
                    if(phase==Phase.FIRST&&collectionKind!=null&&feedSeconds>0) {
                        if(config.flowerDrainOff()) waitForBetaPark(now,"First feed duration complete; flower drain off (forward-back cycles 0)");
                        else outbound(now,"First feed duration complete");
                    }
                    else if(collectionKind!=null) waitForBetaPark(now,"Shot complete");
                    else park(now,firstWindow?"No first-shot feed before window ended":"Shot complete");
                    break;
                }
                if(run==Run.SHOT_ONLY&&!volleyStarted&&remaining<=parkReserve+requested+MAX_LOOP_GAP_S) {
                    park(now,"Insufficient time for full volley before departure");
                    break;
                }
                o.launcher=config.v[6];
                // After feed starts, finish the volley through camera changes and flywheel dips.
                boolean poseForFeed=settled||(mustShoot()&&departed&&now-phaseAt>1.0);
                // Primary first volley (v54): its cell starts UP and nothing may stop it, so a launcher
                // that never reads 96 % still feeds after the normal spin-up time.
                boolean primaryFirst=mustShoot()&&!config.secondary&&phase==Phase.FIRST;
                boolean speedOk=readySpeed||(primaryFirst&&now-phaseAt>=PollenConfig.LAUNCHER_SPINUP_SECONDS);
                o.feed=volleyStarted||(poseForFeed&&speedOk&&(!checksHive||hiveUpLatched));
                if(o.feed&&!volleyStarted) { volleyX=x; volleyY=y; volleyH=h; }
                if(o.feed) {
                    // Paced feed (off at 0 %): hold the agitator while the launcher recovers from a ball.
                    double pause=config.feedPauseFraction();
                    if(pause<=0||readySpeed) feedPaused=false;
                    else if(velocity<config.v[6]*pause) feedPaused=true;
                    o.agitator=feedPaused?0:config.v[12]; wasFeeding=true;
                    if(feedPaused) decision+=" feed_paused=true";
                }
                if(o.feed) reason=checksHive?"Feeding: hive UP latched"
                        :phase==Phase.SECOND?"Feeding return volley: cell already UP, no camera wait":"Feeding primary preload";
                else if(!settled) reason="Not settled at shooting pose";
                else if(!readySpeed) reason="Waiting for measured launcher speed";
                else reason=hiveReady?"Timing confirmed hive UP":hive.reason;
                break;
            case FLOWER: case GARDEN:
                if(collection!=null) {
                    PollenCollectionSequence.Output c=collection.tick(now,x,y,h,odometryReady);
                    motionPhase=c.phase;
                    if(c.contactLimit!=null) decision+=" CONTACT LIMIT: "+c.contactLimit;
                    if(!c.done&&c.fault==null) {
                        o.forward=c.forward; o.strafe=c.strafe; o.turn=c.turn;
                        if(config.v[0]!=1) { o.intake=c.intake; o.drainerDown=c.drainerDown; }
                        break;
                    }
                    returnToShot(now,config.collectionReturn(x,y),c.done?"Collection complete":c.fault);
                    if(!config.angled||!config.secondary||c.fault!=null) break;
                    // Angled secondary starts the return on the same tick as scoop up and spin-up.
                }
                // Garden + Return uses the same motion path for its standalone route.
            case TO_SHOT: case OUTBOUND: case RETURN: case ROOM: case PARK:
                // The nudge is one short leg; if it cannot finish quickly, skip the volley and park as before.
                if(phase==Phase.RETURN&&nudged&&now-phaseAt>1.5) {
                    if(mustShoot()) {
                        reason=String.format(Locale.US,"Return nudge did not reach the spot; shooting from %.2f in off",distanceFromShot);
                        settledAt=Double.NaN; enter(Phase.SECOND,now);
                    } else waitForBetaPark(now,"Return nudge did not reach the shot spot; second shot skipped");
                    break;
                }
                if((phase==Phase.RETURN||phase==Phase.TO_SHOT)&&atShotPosition&&headingNeedsCorrection) {
                    alignForReturnShot(now,headingError);
                    break;
                }
                PollenHiveMotion.Output m=motion.tick(now,x,y,h); motionPhase=m.phase; decision+=" "+m.detail;
                if(m.fault!=null) {
                    // Primary (v54): never lose the first volley; shoot from wherever the drive stopped.
                    if(phase==Phase.TO_SHOT&&mustShoot()&&!config.secondary) {
                        reason="Drive to the shot spot failed ("+m.fault+"); shooting from here";
                        aimGivenUp=true; settledAt=Double.NaN; enter(Phase.FIRST,now);
                        break;
                    }
                    return stop(m.fault);
                }
                if(m.done) {
                    if(phase==Phase.OUTBOUND) {
                        collection=new PollenCollectionSequence(collectionKind,config.collectionSettings(),x,y,0);
                        enter(collectionKind==PollenCollectionSequence.Kind.FLOWER?Phase.FLOWER:Phase.GARDEN,now);
                    } else if(phase==Phase.GARDEN||phase==Phase.RETURN||phase==Phase.TO_SHOT) {
                        enter(Phase.SETTLE,now); hive.reset(); settledAt=Double.NaN;
                    }
                    else if(phase==Phase.ROOM) {
                        enter(Phase.WAIT,now);
                        double reserve=config.cornerParkReserve(x,y);
                        reason=String.format(Locale.US,"In the corner; park with %.2f s left",reserve);
                        // Late arrival: park on this same tick rather than one loop later.
                        if(remaining<=reserve) park(now,"Reached the corner late: park now");
                    }
                    else return stop("Finished departure");
                } else {
                    o.forward=m.forward; o.strafe=m.strafe; o.turn=m.turn;
                    if(m.collect&&config.v[0]!=1) o.intake=1;
                }
                break;
            case WAIT:
                // Sit still in the corner, out of the hero's way, until park time.
                if(remaining<=config.cornerParkReserve(x,y))
                    park(now,String.format(Locale.US,"%.1f s left: park",remaining));
                break;
            case SETTLE:
                if(collectionKind!=null&&headingNeedsCorrection&&!aimGivenUp
                        &&(atShotPosition||(mustShoot()&&distanceFromShot<=MUST_SHOOT_ALIGN_IN))) {
                    alignForReturnShot(now,headingError);
                } else if(settled) {
                    if(run==Run.GARDEN_RETURN) return stop("Garden returned and settled at S");
                    enter(firstShotDone||!angledStart()?Phase.SECOND:Phase.FIRST,now);
                }
                else if(mustShoot()&&now-phaseAt>1.5) {
                    reason=String.format(Locale.US,"Not settled after 1.5 s, %.2f in from the spot; shooting anyway",distanceFromShot);
                    enter(firstShotDone||!angledStart()?Phase.SECOND:Phase.FIRST,now);
                }
                else if(!firstShotDone&&angledStart()&&now-phaseAt>1.5)
                    skipAngledPreload(now,"Shot spot did not settle; first shot skipped");
                // Fast mode: stopped just off the spot (0.5-3 in). One short leg back instead of losing the volley.
                else if(config.fast&&!nudged&&collectionKind!=null&&!atShotPosition&&distanceFromShot<=3
                        &&speed<=1.5&&now-phaseAt>=.3) {
                    nudged=true;
                    motion=new PollenHiveMotion(new PollenHiveRoute.Node[]{config.returnNudge()});
                    reason=String.format(Locale.US,"Stopped %.2f in off the shot spot; nudging back",distanceFromShot);
                    enter(Phase.RETURN,now);
                }
                // Off the shot pose: skip the second shot but still park (park() stops in practice runs).
                else if(now-phaseAt>1.5) {
                    if(collectionKind!=null) waitForBetaPark(now,"Return pose did not settle; second shot skipped");
                    else park(now,"Return pose did not settle; second shot skipped");
                }
                break;
            case ALIGN_SHOT:
                if(mustShoot()&&(distanceFromShot>MUST_SHOOT_ALIGN_IN||now>alignUntil)) {
                    aimGivenUp=true; settledAt=Double.NaN;
                    reason=String.format(Locale.US,"Aim turn incomplete (%.1f deg off); shooting anyway",headingError);
                    enter(firstShotDone||!angledStart()?Phase.SECOND:Phase.FIRST,now);
                } else if(distanceFromShot>1||now>alignUntil) {
                    if(!firstShotDone&&angledStart()) skipAngledPreload(now,"Aim turn failed; first shot skipped");
                    else waitForBetaPark(now,"Return heading correction failed; second shot skipped");
                } else if(Math.abs(headingError)<=RETURN_ALIGNMENT_DEG&&omega<=Math.toRadians(10)) {
                    settledAt=Double.NaN;
                    reason="Heading corrected; settling for return shot";
                    enter(Phase.SETTLE,now);
                } else {
                    o.turn=PollenOdometryDrive.turnPower(shotH,config.v[8],h,turnRate,RETURN_ALIGNMENT_DEG);
                }
                break;
            default: break;
        }
        if(collectionKind!=null&&config.v[0]!=1&&(phase==Phase.TO_SHOT||phase==Phase.RETURN||phase==Phase.SETTLE||phase==Phase.ALIGN_SHOT||phase==Phase.SECOND))
            o.launcher=config.v[6];
        if(phase!=Phase.DONE) {
            applyClearOut(o,now);
            if(config.v[0]!=1)
                o.intake=PollenConfig.autoIntakePower(o.forward,o.strafe,o.turn,
                        o.feed||now<clearOutUntil||o.intake!=0);
        }
        o.finished=phase==Phase.DONE;
        o.phase=phase+((phase==Phase.TO_SHOT||phase==Phase.OUTBOUND||phase==Phase.GARDEN||phase==Phase.FLOWER
                ||phase==Phase.RETURN||phase==Phase.ROOM||phase==Phase.PARK)?": "+motionPhase:"");
        o.reason=reason; o.decision=decision;
        return o;
    }
    /** Clear-out: after a match volley, the agitator keeps delivering and the launcher keeps
     * shooting for the same set seconds, so the robot is empty before it collects again. Both stop
     * on the same tick, so the agitator never pushes a ball into a stopped launcher. Only the
     * agitator and launcher are set here. The common intake rule treats clear-out as shooting. */
    private void startClearOut(double now) {
        if(config.v[0]==1) return;
        clearOutUntil=now+config.clearOutSeconds();
    }
    private void endClearOut() { clearOutUntil=Double.NEGATIVE_INFINITY; }
    private void applyClearOut(Output o,double now) {
        if(now>=clearOutUntil) return;
        o.agitator=Math.max(o.agitator,config.v[12]);
        o.launcher=Math.max(o.launcher,config.v[6]);
        decision+=String.format(Locale.US," clear_out_s_left=%.2f",clearOutUntil-now);
    }
    private void alignForReturnShot(double now,double headingError) {
        alignUntil=now+PollenOdometryDrive.turnTimeoutSeconds(Math.abs(headingError),config.v[8]);
        reason=String.format(Locale.US,"Correcting heading to %.1f degrees",Math.toDegrees(shotH));
        enter(Phase.ALIGN_SHOT,now);
    }
    /** The agitator meters balls into the flywheel. Stop both shooting outputs on the
     * transition tick, then drive/collect without waiting for flywheel spin-down. */
    private void garden(double now,String why) {
        double need=PollenHiveRoute.planned(config.garden())+2;
        if(27-(now-began)<need) { park(now,"Garden skipped: not enough estimated cycle time"); return; }
        reason=why+"; garden cycle budget passed";
        motion=new PollenHiveMotion(config.garden()); enter(Phase.GARDEN,now);
    }
    private void skipAngledPreload(double now,String why) {
        // Secondary can skip the return camera check only after a confirmed first volley.
        // Its four preloads are still onboard, so skip the flower and keep the scheduled park.
        if(config.secondary) waitForBetaPark(now,why);
        else outbound(now,why);
    }
    private void outbound(double now,String why) {
        firstShotDone=true;
        reason=why+"; starting collection trip";
        motion=new PollenHiveMotion(config.collectionOutbound());
        enter(Phase.OUTBOUND,now);
    }
    private void returnToShot(double now,PollenHiveRoute.Node[] home,String why) {
        reason=why+"; returning to shooting position";
        collection=null;
        motion=new PollenHiveMotion(home);
        enter(Phase.RETURN,now);
    }
    private void enter(Phase p,double now) {
        phase=p;
        if(p==Phase.TO_SHOT||p==Phase.RETURN) aimGivenUp=false; // a new approach gets its own aim turn
        if(p==Phase.TO_SHOT||p==Phase.OUTBOUND||p==Phase.GARDEN||p==Phase.ROOM||p==Phase.WAIT||p==Phase.PARK) departed=true;
        motionPhase="";
        phaseAt=now;
        feedSeconds=0;
        wasFeeding=false;
        hiveUpSince=Double.NaN;
        hiveUpLatched=false;
    }
    /** Seconds kept free before 27 s for leaving the shooting spot: the park route, or for make
     * room the corner route plus the late-park time. Practice runs never leave, so 0. */
    private double departureReserve() {
        if(!config.makeRoom||run!=Run.MATCH) return 0;
        return config.cornerDepartureReserve();
    }
    private void waitForBetaPark(double now,String why) {
        if(!config.parkEnabled()) { stop(why); return; }
        reason=why+"; waiting for park deadline";
        enter(Phase.PARK_WAIT,now);
    }
    private void startBetaPark(double now,String why,PollenHiveRoute.Node[] approach) {
        PollenHiveRoute.Node[] route=config.collectionPark();
        if(approach!=null) {
            PollenHiveRoute.Node[] combined=Arrays.copyOf(approach,approach.length+route.length);
            System.arraycopy(route,0,combined,approach.length,route.length);
            route=combined;
        }
        collection=null;
        endClearOut(); // the park deadline wins over clearing
        reason=why;
        motion=new PollenHiveMotion(route);
        enter(Phase.PARK,now);
    }
    private void park(double now,String why) {
        if(run!=Run.MATCH) { stop(why); return; }
        endClearOut();
        if(config.makeRoom&&phase!=Phase.WAIT) {
            // Make room: get out of the hero's way first; WAIT parks with the set seconds left.
            reason=why+"; making room in the corner"; motion=new PollenHiveMotion(config.corner()); enter(Phase.ROOM,now); return;
        }
        reason=why;
        motion=new PollenHiveMotion(config.makeRoom?config.cornerPark():config.collectionPark());
        enter(Phase.PARK,now);
    }
}
