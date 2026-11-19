package org.firstinspires.ftc.teamcode.pollen;

/** Nonblocking measured moves/turns. Strict arrival by default. */
public final class PollenHiveMotion {
    private static final double CONTACT_PROGRESS_IN = .15;
    private static final double MAX_TRANSLATION_HEADING_ERROR = Math.toRadians(12);
    enum Contact { STRICT, ALLOWED, DRIVE_UNTIL_BLOCKED }
    public static final class Output {
        public double forward,strafe,turn;
        public boolean done,collect,contactLimit;
        public String phase="",fault,detail="";
    }
    private final PollenHiveRoute.Node[] nodes;
    private final Contact contact;
    private int index;
    private double began,previous=Double.NaN,px,py,ph,progressAt,progressX,progressY,progressH,progressDistance;
    private boolean newNode=true;
    private String fault,detail="";
    public PollenHiveMotion(PollenHiveRoute.Node[] nodes) { this(nodes,Contact.STRICT); }
    PollenHiveMotion(PollenHiveRoute.Node[] nodes,Contact contact) {
        this.nodes=nodes; this.contact=contact;
    }
    public Output tick(double now,double x,double y,double h) {
        Output o=new Output();
        if(fault!=null) { o.fault=fault; o.detail=detail; return o; }
        if(!Double.isFinite(now)||!Double.isFinite(x)||!Double.isFinite(y)||!Double.isFinite(h)) return fail("Invalid pose/time");
        if(index>=nodes.length) { o.done=true; return o; }
        double dt=now-previous;
        if(Double.isFinite(previous)&&(dt<0||dt>.25)) return fail("Motion loop stalled");
        PollenHiveRoute.Node n=nodes[index]; o.phase=n.name;
        // For a wall drive, node x/y is a unit vector in route coordinates.
        boolean wallDrive=contact==Contact.DRIVE_UNTIL_BLOCKED;
        double distance=Math.hypot(n.x-x,n.y-y), error=PollenOdometryDrive.wrap(h-n.h);
        double travel=wallDrive&&!newNode?(x-progressX)*n.x+(y-progressY)*n.y:0;
        double remaining=wallDrive?-travel:distance;
        if(newNode) {
            began=progressAt=now; progressX=x; progressY=y; progressH=h;
            progressDistance=remaining; newNode=false;
        }
        double vx=dt>0?(x-px)/dt:0, vy=dt>0?(y-py)/dt:0;
        double omega=dt>0?PollenOdometryDrive.wrap(h-ph)/dt:Double.POSITIVE_INFINITY;
        previous=now; px=x; py=y; ph=h;
        double elapsed=now-began;
        detail=wallDrive
                ? String.format(java.util.Locale.US,"node=%s travel_in=%.2f target_deg=%.2f heading_error_deg=%.2f node_s=%.3f no_progress_s=%.3f",
                        n.name,travel,Math.toDegrees(n.h),Math.toDegrees(error),elapsed,now-progressAt)
                : String.format(java.util.Locale.US,"node=%s target_in=%.2f/%.2f target_deg=%.2f distance_in=%.2f heading_error_deg=%.2f node_s=%.3f no_progress_s=%.3f",
                        n.name,n.x,n.y,Math.toDegrees(n.h),distance,Math.toDegrees(error),elapsed,now-progressAt);
        o.detail=detail;
        if(n.seconds>0) {
            if(distance>.75||Math.abs(error)>Math.toRadians(3)) return fail("Collection hold pose drift");
            if(elapsed>=n.seconds) return advance(o);
            o.collect=n.collect; return o;
        }
        boolean settled=dt>0&&Math.hypot(vx,vy)<=1.5&&Math.abs(omega)<=Math.toRadians(10);
        if(!wallDrive&&distance<=.5&&Math.abs(error)<=Math.toRadians(2)&&settled) return advance(o);
        if(n.turn) {
            if(distance>1) return fail("Turn drifted out of position");
            double timeout=PollenOdometryDrive.turnTimeoutSeconds(180,n.power);
            if(elapsed>timeout) return fail(n.name+" turn timeout");
            if(Math.abs(error)>Math.toRadians(2)) {
                double power=Math.min(PollenOdometryDrive.rampedTurnPower(elapsed,n.power),
                        Math.max(.12,.08*Math.sqrt(Math.abs(Math.toDegrees(error)))));
                o.turn=Math.copySign(power,error);
            }
        } else if(wallDrive) {
            double power=PollenOdometryDrive.rampedPower(elapsed,n.power);
            double cos=Math.cos(h), sin=Math.sin(h);
            o.forward=power*(cos*n.x+sin*n.y);
            o.strafe=power*(sin*n.x-cos*n.y);
        } else {
            if(elapsed>12) return fail(n.name+" move timeout");
            PollenOdometryDrive.Command c=PollenOdometryDrive.command(n.x,n.y,
                    PollenOdometryDrive.rampedPower(elapsed,n.power),x,y,h,vx,vy);
            o.forward=c.forward; o.strafe=c.strafe;
        }
        if(!n.turn) {
            o.turn=Math.max(-.25,Math.min(.25,error));
            if(Math.abs(error)>MAX_TRANSLATION_HEADING_ERROR) { o.forward=0; o.strafe=0; }
        }
        if(contact!=Contact.STRICT&&!n.turn) {
            // Check even near the target, where a blocked intake or drainer can still push.
            if(progressDistance-remaining>=CONTACT_PROGRESS_IN) { progressAt=now; progressDistance=remaining; }
            else if(now-progressAt>PollenOdometryDrive.NO_PROGRESS_SECONDS) {
                double headingTolerance=wallDrive?MAX_TRANSLATION_HEADING_ERROR:Math.toRadians(2);
                if(Math.abs(error)>headingTolerance||(!wallDrive&&!settled)) return fail(n.name+" did not settle");
                if(settled) {
                    o.contactLimit=true;
                    return advance(o);
                }
            }
            o.collect=n.collect;
            return o;
        }
        double progress=n.turn?Math.abs(Math.toDegrees(PollenOdometryDrive.wrap(h-progressH))):Math.hypot(x-progressX,y-progressY);
        boolean commanding=Math.abs(o.forward)+Math.abs(o.strafe)+Math.abs(o.turn)>.01;
        boolean far=n.turn?Math.abs(error)>Math.toRadians(5):distance>2;
        if(progress>=(n.turn?2:.5)||!commanding||!far) { progressAt=now; progressX=x; progressY=y; progressH=h; }
        else if(now-progressAt>.75) return fail(n.name+" no odometry progress");
        o.collect=n.collect; return o;
    }
    private Output advance(Output o) { index++; newNode=true; o.forward=o.strafe=o.turn=0; o.collect=false; return o; }
    private Output fail(String message) { fault=message; Output o=new Output(); o.fault=message; o.detail=detail; return o; }
}
