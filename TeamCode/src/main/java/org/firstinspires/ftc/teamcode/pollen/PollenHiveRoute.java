package org.firstinspires.ftc.teamcode.pollen;

import java.util.*;

/** Garden route derived from docs/POLLEN_FIELD_LAYOUT.md, red primary start; blue rotates 180 deg.
 * Route coordinates remain x toward hive, y left, h CCW from START, although front faces the wall.
 * Points are robot centers. 18x18 footprint; the mounted intake must fit that footprint.
 */
public final class PollenHiveRoute {
    public static final String[] NAMES={"Profile slot", "Execution 0=DRY 1=DRIVE 2=LIVE", "Start delay s",
        "First feed s", "Second feed s", "First shot window s", "Launcher ticks/s", "Travel power",
        "Turn power", "Collection power", "Collection hold s", "Feed intake power", "Feed agitator power",
        "Clearance forward cm", "First turn left cm", "Garden turn left cm", "Pickup left cm",
        "Pickup forward cm", "Transit heading deg", "Pickup heading deg", "Pickup drive power",
        "Park 0=no 1=yes", "Park1 forward cm", "Park1 left cm", "Park2 forward cm", "Park2 left cm",
        "Park3 forward cm", "Park3 left cm"};
    public final double[] v=new double[NAMES.length];
    public final boolean secondary;
    public PollenHiveRoute(boolean secondary) { this.secondary=secondary; defaults(); }
    public void defaults() {
        double startX=PollenFieldGeometry.RED_HIVE_X_INCHES;
        double wall=PollenFieldGeometry.END_WALL_INCHES;
        double gardenTurnX=-wall+15; // 15 inches from side wall, beyond the 12.73-inch turn radius.
        double pickupX=-wall+PollenFieldGeometry.ROBOT_HALF_INCHES+1; // 1-inch body/wall gap.
        double[] d={secondary?2:1,0,0,1.5,1.5,5,1180,.60,.40,.5,.5,.5,1,
            6*2.54,(startX-(-30))*2.54,(startX-gardenTurnX)*2.54,(startX-pickupX)*2.54,
            1*2.54,-90,0,.20,1,0,0,0,0,0,0};
        System.arraycopy(d,0,v,0,d.length);
        // Nominal CAD park points, NOT the legacy timed field-tuned distance shortcuts.
        if(secondary) {
            v[22]=8*2.54; v[23]=0;
            v[24]=v[22]; v[25]=0;
            v[26]=(PollenFieldGeometry.SUPPORT_PARK_Z_INCHES-PollenFieldGeometry.supportStartZInches())*2.54;
            v[27]=(PollenFieldGeometry.PARK_X_INCHES-PollenFieldGeometry.SUPPORT_START_X_INCHES)*2.54;
        } else {
            v[22]=10; v[23]=0; v[24]=10; v[25]=25*2.54;
            v[26]=(PollenFieldGeometry.END_WALL_INCHES-9-PollenFieldGeometry.PRIMARY_PARK_Z_INCHES)*2.54;
            v[27]=(startX-PollenFieldGeometry.PARK_X_INCHES)*2.54;
        }
    }
    public void edit(int i,int delta) {
        double step=i<=1||i==21?1:i==6?20:i>=18&&i<=19?5:i>=13&&i<=17||i>=22?1:
                i==7||i==8||i==9||i==11||i==12||i==20?.05:.5;
        v[i]+=delta*step;
        if(i==0) v[i]=clamp(v[i],1,8);
        else if(i==1) v[i]=clamp(v[i],0,2);
        else if(i==21) v[i]=clamp(v[i],0,1);
        else if(i==6) v[i]=clamp(v[i],500,2500);
        else if(i==7||i==8||i==20) v[i]=clamp(v[i],.1,.8);
        else if(i==9||i==11||i==12) v[i]=clamp(v[i],0,1);
        else if(i==18||i==19) v[i]=clamp(v[i],-180,180);
        else if(i>=13) v[i]=clamp(v[i],-350,350);
        else v[i]=clamp(v[i],0,20);
    }
    private static double clamp(double x,double a,double b) { return Math.max(a,Math.min(b,x)); }
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
        if(v[0]<1||v[0]>8||v[1]<0||v[1]>2||v[6]<500||v[6]>2500)
            throw new IllegalArgumentException("Invalid slot/mode/velocity");
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
    public Properties properties() {
        Properties p=new Properties(); p.setProperty("version","1");
        for(int i=0;i<v.length;i++) if(i!=1) p.setProperty("v"+i,""+v[i]);
        return p;
    }
    public void load(Properties p) {
        if(!"1".equals(p.getProperty("version"))) throw new IllegalArgumentException("Settings version mismatch");
        double[] old=v.clone();
        try { for(int i=0;i<v.length;i++) if(i!=1) v[i]=Double.parseDouble(p.getProperty("v"+i)); validate(); }
        catch(RuntimeException ex) { System.arraycopy(old,0,v,0,v.length); throw ex; }
        v[1]=0; // Never persist a motor-enabled selection.
    }
}
