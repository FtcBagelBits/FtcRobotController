package org.firstinspires.ftc.teamcode.pollen;

/** Garden route derived from docs/POLLEN_FIELD_LAYOUT.md, red primary start; blue rotates 180 deg.
 * Route coordinates remain x toward hive, y left, h CCW from START, although front faces the wall.
 * Points are robot centers. 18x18 footprint; the mounted intake must fit that footprint.
 * The numbers and their defaults live in PollenHiveSettings.ROUTE; v[i] is row i of that table.
 */
public final class PollenHiveRoute {
    public final PollenHiveSettings.Values values;
    public final double[] v;
    public final boolean secondary;
    public PollenHiveRoute(boolean secondary) {
        this.secondary=secondary; values=new PollenHiveSettings.Values(PollenHiveSettings.ROUTE,secondary); v=values.v;
    }
    /** Side row: 0 = detect from the tags behind the robot, 1 = red, 2 = blue. */
    public int side() { return (int)v[1]; }
    public static final class Node {
        public final String name;
        public final double x,y,h,power,seconds;
        public final boolean turn,collect;
        Node(String name,double x,double y,double h,double power,boolean turn,boolean collect,double seconds) {
            this.name=name; this.x=x; this.y=y; this.h=Math.toRadians(h); this.power=power;
            this.turn=turn; this.collect=collect; this.seconds=seconds;
        }
    }
    private Node move(String name,double xCm,double yCm,double h,boolean collect) {
        return new Node(name,xCm/2.54,yCm/2.54,h,collect?v[20]:v[7],false,collect,0);
    }
    private Node turn(String name,double xCm,double yCm,double h) {
        return new Node(name,xCm/2.54,yCm/2.54,h,v[8],true,false,0);
    }
    public Node[] garden() {
        double f=v[13], a=v[14], b=v[15], c=v[16], near=v[17], transit=v[18], pickup=v[19];
        return new Node[]{
            move("CLEAR END WALL",f,0,0,false),
            move("REACH FIRST TURN",f,a,0,false),
            turn("FRONT TOWARD GARDEN",f,a,transit),
            move("TRAVEL FRONT FIRST",f,b,transit,false),
            turn("FRONT TOWARD BALL ROW",f,b,pickup),
            move("ALIGN INTAKE",f,c,pickup,false),
            move("COLLECT TOWARD WALL",near,c,pickup,true),
            new Node("COLLECT HOLD",near/2.54,c/2.54,pickup,0,false,true,Math.max(.02,v[10])),
            move("BACK AWAY FROM WALL",f,c,pickup,false),
            move("CLEAR SIDE WALL",f,b,pickup,false),
            turn("RESTORE SHOOT HEADING",f,b,0),
            move("RETURN ACROSS",f,0,0,false),
            move("RETURN SHOOT POSITION",0,0,0,false)};
    }
    public Node[] park() {
        if(v[21]==0) return new Node[0];
        return new Node[]{move("PARK CLEAR",v[22],v[23],0,false),
            move("PARK CORNER",v[24],v[25],0,false),move("PARK",v[26],v[27],0,false)};
    }
    public static double planned(Node[] nodes) {
        double x=0,y=0,h=0,total=0;
        for(Node n:nodes) {
            if(n.seconds>0) total+=n.seconds;
            else if(n.turn) total+=PollenOdometryDrive.turnPlannedSeconds(Math.toDegrees(PollenOdometryDrive.wrap(n.h-h)),n.power);
            else total+=PollenOdometryDrive.plannedSeconds(Math.hypot(n.x-x,n.y-y)*2.54,n.power);
            x=n.x; y=n.y; h=n.h;
        }
        return total+nodes.length*.04;
    }
    public void validate() {
        for(double d:v) if(!Double.isFinite(d)) throw new IllegalArgumentException("Nonfinite setting");
        if(v[0]<0||v[0]>2||v[1]<0||v[1]>2||v[6]<500||v[6]>2500)
            throw new IllegalArgumentException("Invalid execution/side/velocity");
        for(int i:new int[]{7,8,20}) if(v[i]<.1||v[i]>.8) throw new IllegalArgumentException("Invalid drive power");
        for(int i:new int[]{9,11,12}) if(v[i]<0||v[i]>1) throw new IllegalArgumentException("Invalid mechanism power");
        for(int i:new int[]{2,3,4,5,10}) if(v[i]<0||v[i]>20) throw new IllegalArgumentException("Invalid duration");
        if(v[21]!=0&&v[21]!=1) throw new IllegalArgumentException("Invalid park choice");
        if(!secondary) checkGarden();
        checkPark();
        if(planned(park())+v[2]+1>=27) throw new IllegalArgumentException("Departure exceeds time budget");
    }
    private void checkGarden() {
        // Check nominal rigid footprint continuously along each translation and one-degree turn.
        // Deliberately strict: edits that hit a wall, frame, flower, or cross alliance midline fail INIT.
        double x=0,y=0,h=0;
        for(Node n:garden()) {
            double dh=PollenOdometryDrive.wrap(n.h-h);
            if(!n.turn && Math.abs(dh)>Math.toRadians(.01)) throw new IllegalArgumentException("Translation heading mismatch");
            int steps=n.turn?Math.max(1,(int)Math.ceil(Math.abs(Math.toDegrees(dh)))):
                    Math.max(1,(int)Math.ceil(Math.hypot(n.x-x,n.y-y)/.1));
            if(n.turn && Math.hypot(n.x-x,n.y-y)>.01) throw new IllegalArgumentException("Turn must stay in place");
            for(int j=0;j<=steps;j++) {
                double t=(double)j/steps;
                checkFootprint(x+(n.x-x)*t,y+(n.y-y)*t,h+dh*t,n.name);
            }
            x=n.x; y=n.y; h=n.h;
        }
    }
    private void checkPark() {
        double x=0,y=0;
        for(Node n:park()) {
            int steps=Math.max(1,(int)Math.ceil(Math.hypot(n.x-x,n.y-y)/.1));
            for(int j=0;j<=steps;j++) {
                double t=(double)j/steps, f=x+(n.x-x)*t, l=y+(n.y-y)*t;
                double fieldX=secondary?PollenFieldGeometry.SUPPORT_START_X_INCHES+l:PollenFieldGeometry.RED_HIVE_X_INCHES-l;
                double fieldZ=secondary?PollenFieldGeometry.supportStartZInches()+f:PollenFieldGeometry.END_WALL_INCHES-9-f;
                checkFieldFootprint(fieldX,fieldZ,0,n.name);
            }
            x=n.x; y=n.y;
        }
    }
    private static void checkFootprint(double f,double l,double h,String label) {
        double x=PollenFieldGeometry.RED_HIVE_X_INCHES-l;
        double z=PollenFieldGeometry.END_WALL_INCHES-9-f;
        checkFieldFootprint(x,z,h,label);
    }
    private static void checkFieldFootprint(double x,double z,double h,String label) {
        double extent=9*(Math.abs(Math.cos(h))+Math.abs(Math.sin(h)));
        if(x-extent< -70.585-1e-7 || x+extent>1e-7 || z+extent>70.585+1e-7 || z-extent< -70.585)
            throw new IllegalArgumentException(label+": footprint crosses wall/midline");
        // Conservative axis-aligned bounding box of the rotated robot against saved CAD envelopes.
        double[][] obstacles={{-24.729,24.742,-25.071,25.071},{-71.794,-65.630,20.058,26.727},
            {-26.727,-20.058,-71.794,-65.630}};
        for(double[] b:obstacles) if(x+extent>b[0]&&x-extent<b[1]&&z+extent>b[2]&&z-extent<b[3])
            throw new IllegalArgumentException(label+": footprint intersects field obstacle");
    }
    /** Every setting with its key, for the robot log. */
    @Override public String toString() {
        StringBuilder b=new StringBuilder();
        for(int i=0;i<v.length;i++) b.append(i==0?"":", ").append(values.rows[i].key).append('=').append(v[i]);
        return b.toString();
    }
}
