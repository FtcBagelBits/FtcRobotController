package org.firstinspires.ftc.teamcode.pollen;

/** Nonblocking measured moves/turns. Strict arrival; never accept the legacy 2-inch shortcut
 * near the garden walls. Translation may hold a nonzero heading for the front intake. */
public final class PollenHiveMotion {
    public static final class Output {
        public double forward,strafe,turn;
        public boolean done,collect;
        public String phase="",fault;
    }
    private final PollenHiveRoute.Node[] nodes;
    private int index;
    private double began,previous=Double.NaN,px,py,ph,progressAt,progressX,progressY,progressH;
    private boolean newNode=true;
    private String fault;
    public PollenHiveMotion(PollenHiveRoute.Node[] nodes) { this.nodes=nodes; }
    public Output tick(double now,double x,double y,double h) {
        Output o=new Output();
        if(fault!=null) { o.fault=fault; return o; }
        if(!Double.isFinite(now)||!Double.isFinite(x)||!Double.isFinite(y)||!Double.isFinite(h)) return fail("Invalid pose/time");
        if(index>=nodes.length) { o.done=true; return o; }
        double dt=now-previous;
        if(Double.isFinite(previous)&&(dt<0||dt>.25)) return fail("Motion loop stalled");
        PollenHiveRoute.Node n=nodes[index]; o.phase=n.name;
        if(newNode) { began=progressAt=now; progressX=x; progressY=y; progressH=h; newNode=false; }
        double vx=dt>0?(x-px)/dt:0, vy=dt>0?(y-py)/dt:0;
        double omega=dt>0?PollenOdometryDrive.wrap(h-ph)/dt:Double.POSITIVE_INFINITY;
        previous=now; px=x; py=y; ph=h;
        double distance=Math.hypot(n.x-x,n.y-y), error=PollenOdometryDrive.wrap(h-n.h);
        double elapsed=now-began;
        if(n.seconds>0) {
            if(distance>.75||Math.abs(error)>Math.toRadians(3)) return fail("Collection hold pose drift");
            if(elapsed>=n.seconds) return advance(o);
            o.collect=n.collect; return o;
        }
        boolean settled=dt>0&&Math.hypot(vx,vy)<=1.5&&Math.abs(omega)<=Math.toRadians(10);
        if(distance<=.5&&Math.abs(error)<=Math.toRadians(2)&&settled) return advance(o);
        if(n.turn) {
            if(distance>1) return fail("Turn drifted out of position");
            double timeout=PollenOdometryDrive.turnTimeoutSeconds(180,n.power);
            if(elapsed>timeout) return fail(n.name+" turn timeout");
            if(Math.abs(error)>Math.toRadians(2)) {
                double power=Math.min(PollenOdometryDrive.rampedTurnPower(elapsed,n.power),
                        Math.max(.12,.08*Math.sqrt(Math.abs(Math.toDegrees(error)))));
                o.turn=Math.copySign(power,error);
            }
        } else {
            if(elapsed>12) return fail(n.name+" move timeout");
            PollenOdometryDrive.Command c=PollenOdometryDrive.command(n.x,n.y,
                    PollenOdometryDrive.rampedPower(elapsed,n.power),x,y,h,vx,vy);
            o.forward=c.forward; o.strafe=c.strafe;
            o.turn=Math.max(-.25,Math.min(.25,error));
            if(Math.abs(error)>Math.toRadians(12)) { o.forward=0; o.strafe=0; }
        }
        double progress=n.turn?Math.abs(Math.toDegrees(PollenOdometryDrive.wrap(h-progressH))):Math.hypot(x-progressX,y-progressY);
        boolean commanding=Math.abs(o.forward)+Math.abs(o.strafe)+Math.abs(o.turn)>.01;
        boolean far=n.turn?Math.abs(error)>Math.toRadians(5):distance>2;
        if(progress>=(n.turn?2:.5)||!commanding||!far) { progressAt=now; progressX=x; progressY=y; progressH=h; }
        else if(now-progressAt>.75) return fail(n.name+" no odometry progress");
        o.collect=n.collect; return o;
    }
    private Output advance(Output o) { index++; newNode=true; o.forward=o.strafe=o.turn=0; o.collect=false; return o; }
    private Output fail(String message) { fault=message; Output o=new Output(); o.fault=message; return o; }
}
