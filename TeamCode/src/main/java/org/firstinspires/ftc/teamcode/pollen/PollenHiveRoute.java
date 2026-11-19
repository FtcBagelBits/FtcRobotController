package org.firstinspires.ftc.teamcode.pollen;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/** Garden route derived from docs/POLLEN_FIELD_LAYOUT.md, red primary start; blue rotates 180 deg.
 * Route coordinates remain x toward hive, y left, h CCW from START, although front faces the wall.
 * Points are robot centers. 18x18 footprint; the mounted intake must fit that footprint.
 * The numbers and their defaults live in PollenHiveSettings.ROUTE; v[i] is row i of that table.
 */
public final class PollenHiveRoute {
    public final PollenHiveSettings.Values values;
    public final double[] v;
    public final boolean secondary;
    /** MAKE ROOM secondary (Play 2): after the shot, wait in the far corner, then park late. */
    public final boolean makeRoom;
    /** Rows used only by the make-room secondary (indices into v[]). */
    public static final int ROOM_FORWARD=28, ROOM_LEFT=29;
    private static final double CORNER_SETTLE_PER_LEG = .4;
    private static final int COLLECTION_ENTRY_FORWARD=31, COLLECTION_ENTRY_LEFT=32;
    private static final int MATCH_TRAVEL_POWER=33;
    private static final int MATCH_FIRST_FEED=34, MATCH_SECOND_FEED=35;
    private static final int MATCH_GARDEN_PASSES=36, MATCH_FLOWER_SHAKES=37, MATCH_FLOWER_CLEANUP=38;
    private static final int MATCH_FEED_INTAKE=39;
    private static final int MATCH_PARK_SECONDS=40;
    private static final int PARK_MARGIN=41;
    private static final int CLEAR_OUT=42, FEED_PAUSE_PCT=43;
    /** Beta Primary Shooter - Fast: its own defaults (FAST_DEFAULTS) and one return nudge. */
    public final boolean fast;
    /** Fast mode defaults, from the 2026-11-18 traces (docs/research/cycle-time.md): enter the
     * garden 8 in off the end wall and about 3 in short of the side wall instead of 13.4 in and 6.5 in,
     * so the slow contact passes are short; feeds of 2.5 s, since the last launcher dip came by
     * 2.2 s in every 3 s volley and the clear-out catches stragglers. */
    private static final String[][] FAST_DEFAULTS={
        {"collectionEntryForward","20.32"},{"collectionEntryLeft","116"},
        {"matchFirstFeed","2.5"},{"matchSecondFeed","2.5"}};
    private static final String COLLECTION_PREFIX="collection.";
    public PollenHiveRoute(boolean secondary) { this(secondary,false); }
    public PollenHiveRoute(boolean secondary,boolean makeRoom) { this(secondary,makeRoom,false); }
    public PollenHiveRoute(boolean secondary,boolean makeRoom,boolean fast) {
        if(makeRoom&&!secondary) throw new IllegalArgumentException("Make room is a secondary mode");
        if(fast&&secondary) throw new IllegalArgumentException("Fast mode is a primary mode");
        this.secondary=secondary; this.makeRoom=makeRoom; this.fast=fast;
        List<PollenHiveSettings.Row> rows=new ArrayList<>(Arrays.asList(PollenHiveSettings.ROUTE));
        if(fast) for(String[] d:FAST_DEFAULTS) for(int i=0;i<rows.size();i++) {
            PollenHiveSettings.Row r=rows.get(i);
            if(r.key.equals(d[0])) rows.set(i,new PollenHiveSettings.Row(r.key,r.label,
                    Double.parseDouble(d[1]),r.secondary,r.step,r.min,r.max));
        }
        for(PollenHiveSettings.Row row:PollenCollectionSequence.rows(collectionKind())) {
            // Existing beta cycle rows already tune these counts and preserve saved settings.
            if(row.key.equals("repeats")||row.key.equals("shakes")) continue;
            rows.add(new PollenHiveSettings.Row(COLLECTION_PREFIX+row.key,row.label,
                    row.primary,row.secondary,row.step,row.min,row.max));
        }
        values=new PollenHiveSettings.Values(rows.toArray(new PollenHiveSettings.Row[0]),secondary); v=values.v;
    }
    // Calibration uses the saved side row. Each match starts with a fresh driver choice.
    private static final int USE_PROFILE = -1;
    private int matchAlliance = USE_PROFILE;
    public void beginMatchSetup() { matchAlliance = 0; }
    public void selectMatchAlliance(boolean blue) { matchAlliance = blue ? 2 : 1; }
    public boolean needsMatchAlliance() { return matchAlliance == 0; }
    public int side() { return matchAlliance == USE_PROFILE ? (int)v[1] : matchAlliance; }
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
    private Node transit(String name,double xCm,double yCm,double power) {
        return new Node(name,xCm/2.54,yCm/2.54,0,power,false,false,0);
    }
    public Node[] garden() {
        double f=v[13], side=v[16], near=v[17];
        // Keep the starting heading: back clear, strafe to the row, drive front-first
        // into collection, then back out and retrace the corridor to the shooting pose.
        return new Node[]{
            move("CLEAR END WALL",f,0,0,false),
            move("STRAFE TO GARDEN",f,side,0,false),
            move("COLLECT TOWARD WALL",near,side,0,true),
            new Node("COLLECT HOLD",near/2.54,side/2.54,0,0,false,true,Math.max(.02,v[10])),
            move("BACK AWAY FROM WALL",f,side,0,false),
            move("RETURN ACROSS",f,0,0,false),
            move("RETURN SHOOT POSITION",0,0,0,false)};
    }
    /** Back clear of the end wall, then line up with the correct collection entry. */
    public Node[] collectionOutbound() {
        double clear=v[COLLECTION_ENTRY_FORWARD], left=v[COLLECTION_ENTRY_LEFT];
        double power=v[MATCH_TRAVEL_POWER];
        return new Node[]{transit("CLEAR END WALL",clear,0,power),
                transit(secondary?"ALIGN WITH FLOWER":"ALIGN WITH GARDEN",clear,left,power)};
    }
    /** Back clear on the current lane before crossing to the original shooting pose. */
    public Node[] collectionReturn(double x,double y) {
        double clear=Math.max(x*2.54,v[COLLECTION_ENTRY_FORWARD]);
        double power=v[MATCH_TRAVEL_POWER];
        return new Node[]{transit("CLEAR COLLECTION SITE",clear,y*2.54,power),
                transit("RETURN ACROSS",clear,0,power),transit("RETURN SHOOT POSITION",0,0,power)};
    }
    double collectionEstimate() {
        // Approximate process time from v28 field traces with the requested cycle counts.
        double process=secondary?8.3+(v[MATCH_FLOWER_SHAKES]-2)*1.4+(v[MATCH_FLOWER_CLEANUP]-1)*1.05
                :5.5+(v[MATCH_GARDEN_PASSES]-1)*2;
        PollenHiveSettings.Values tuning=collectionSettings();
        double exitX=secondary?(v[COLLECTION_ENTRY_FORWARD]-tuning.get("gap")+tuning.get("back"))/2.54:0;
        double exitY=secondary?v[COLLECTION_ENTRY_LEFT]/2.54:
                PollenFieldGeometry.END_WALL_INCHES-PollenFieldGeometry.ROBOT_HALF_INCHES+PollenFieldGeometry.RED_HIVE_X_INCHES;
        return planned(collectionOutbound())+process+planned(collectionReturn(exitX,exitY),exitX,exitY)+.5;
    }
    PollenCollectionSequence.Kind collectionKind() {
        return secondary?PollenCollectionSequence.Kind.FLOWER:PollenCollectionSequence.Kind.GARDEN;
    }
    PollenHiveSettings.Values collectionSettings() {
        PollenHiveSettings.Values settings=PollenCollectionSequence.settings(collectionKind());
        for(int i=PollenHiveSettings.ROUTE.length;i<v.length;i++) {
            String key=values.rows[i].key.substring(COLLECTION_PREFIX.length());
            settings.v[settings.index(key)]=v[i];
        }
        if(secondary) {
            settings.v[settings.index("shakes")]=v[MATCH_FLOWER_SHAKES];
            settings.v[settings.index("repeats")]=v[MATCH_FLOWER_CLEANUP];
        } else {
            settings.v[settings.index("repeats")]=v[MATCH_GARDEN_PASSES]-1;
        }
        return settings;
    }
    double feedIntake(boolean collectionMatch) { return v[collectionMatch?MATCH_FEED_INTAKE:11]; }
    /** Seconds the agitator and launcher keep running together after a match volley ends, to empty the robot. */
    double clearOutSeconds() { return v[CLEAR_OUT]; }
    /** Paced feed: pause the agitator below this fraction of the launcher target; 0 = never pause. */
    double feedPauseFraction() { return v[FEED_PAUSE_PCT]/100; }
    /** Fast mode: one short leg back onto the shot spot when the return stopped just off it. */
    Node returnNudge() { return transit("RETURN NUDGE",0,0,v[MATCH_TRAVEL_POWER]); }
    double feedDuration(boolean second, boolean collectionMatch) {
        if(collectionMatch) return v[second?MATCH_SECOND_FEED:MATCH_FIRST_FEED];
        return v[second?4:3];
    }
    void validateCollection() {
        double forward=v[COLLECTION_ENTRY_FORWARD], left=v[COLLECTION_ENTRY_LEFT];
        if(forward<20.32||forward>75||(secondary?(left>=0||left< -150):(left<=0||left>150)))
            throw new IllegalArgumentException("Invalid match collection entry");
        if(v[MATCH_TRAVEL_POWER]<.1||v[MATCH_TRAVEL_POWER]>1)
            throw new IllegalArgumentException("Invalid match travel power");
        for(int i:new int[]{MATCH_FIRST_FEED,MATCH_SECOND_FEED})
            if(v[i]<0||v[i]>20) throw new IllegalArgumentException("Invalid beta feed duration");
        if(v[MATCH_FEED_INTAKE]<0||v[MATCH_FEED_INTAKE]>1) throw new IllegalArgumentException("Invalid beta shooting intake power");
        if(v[CLEAR_OUT]<0||v[CLEAR_OUT]>6) throw new IllegalArgumentException("Invalid clear-out time");
        if(v[MATCH_PARK_SECONDS]<.5||v[MATCH_PARK_SECONDS]>20) throw new IllegalArgumentException("Invalid beta measured parking time");
        PollenCollectionSequence.validate(collectionKind(),collectionSettings());
        checkPark(collectionOutbound(),0,0);
    }
    public Node[] park() { return park(v[7]); }
    boolean parkEnabled() { return v[21]!=0; }
    double measuredParkSeconds() { return v[MATCH_PARK_SECONDS]; }
    double parkMargin() { return v[PARK_MARGIN]; }
    double cornerParkReserve(double x,double y) {
        Node[] park=cornerPark();
        return planned(park,x,y)+park.length*CORNER_SETTLE_PER_LEG+parkMargin();
    }
    double cornerDepartureReserve() {
        Node[] approach=corner();
        Node end=approach[approach.length-1];
        // The base estimate understates braking and settling at the corner waypoints in simulation.
        return planned(approach)+approach.length*CORNER_SETTLE_PER_LEG+cornerParkReserve(end.x,end.y);
    }
    public Node[] collectionPark() { return park(v[MATCH_TRAVEL_POWER]); }
    private Node[] park(double power) {
        if(v[21]==0) return new Node[0];
        return new Node[]{transit("PARK CLEAR",v[22],v[23],power),
            transit("PARK CORNER",v[24],v[25],power),transit("PARK",v[26],v[27],power)};
    }
    /** Make room: clear the flower (Park1 forward), slide right along the end wall to the side
     * wall, then back into the corner. */
    public Node[] corner() {
        return new Node[]{move("ROOM CLEAR FLOWER",v[22],0,0,false),
            move("ROOM TO SIDE WALL",v[22],v[ROOM_LEFT],0,false),
            move("ROOM CORNER",v[ROOM_FORWARD],v[ROOM_LEFT],0,false)};
    }
    /** Make room: from the corner straight to the same final parking spot as park(). */
    public Node[] cornerPark() {
        if(v[21]==0) return new Node[0];
        return new Node[]{move("PARK",v[26],v[27],0,false)};
    }
    public static double planned(Node[] nodes) {
        return planned(nodes,0,0);
    }
    /** Estimated seconds for nodes that start at (x, y) inches from START. */
    public static double planned(Node[] nodes,double startX,double startY) {
        double x=startX,y=startY,h=0,total=0;
        for(Node n:nodes) {
            if(n.seconds>0) total+=n.seconds;
            else if(n.turn) total+=PollenOdometryDrive.turnPlannedSeconds(Math.toDegrees(PollenOdometryDrive.wrap(n.h-h)),n.power);
            else total+=PollenOdometryDrive.plannedSeconds(Math.hypot(n.x-x,n.y-y)*2.54,n.power);
            x=n.x; y=n.y; h=n.h;
        }
        return total+nodes.length*.04;
    }
    public void validateForStart() {
        validateForStart(false);
    }
    void validateForStart(boolean collectionMatch) {
        if (needsMatchAlliance()) throw new IllegalArgumentException("Choose alliance: D-pad LEFT red / RIGHT blue");
        validate(collectionMatch);
    }

    public void validate() { validate(false); }

    void validate(boolean collectionMatch) {
        for(double d:v) if(!Double.isFinite(d)) throw new IllegalArgumentException("Nonfinite setting");
        if(parkMargin()<0||parkMargin()>PollenConfig.MAX_PARK_MARGIN) throw new IllegalArgumentException("Invalid park margin");
        if(v[0]<0||v[0]>2||v[1]<0||v[1]>2||v[6]<500||v[6]>2500)
            throw new IllegalArgumentException("Invalid execution/side/velocity");
        for(int i:new int[]{7,8,20}) if(v[i]<.1||v[i]>.8) throw new IllegalArgumentException("Invalid drive power");
        for(int i:new int[]{9,11,12}) if(v[i]<0||v[i]>1) throw new IllegalArgumentException("Invalid mechanism power");
        for(int i:new int[]{2,3,4,5,10}) if(v[i]<0||v[i]>20) throw new IllegalArgumentException("Invalid duration");
        if(v[21]!=0&&v[21]!=1) throw new IllegalArgumentException("Invalid park choice");
        if(!secondary) checkGarden();
        if(makeRoom) {
            Node c=corner()[2];
            checkPark(corner(),0,0);
            checkPark(cornerPark(),c.x,c.y);
            if(cornerDepartureReserve()+v[2]>=27) throw new IllegalArgumentException("Departure exceeds time budget");
        } else {
            checkPark(park(),0,0);
            if(collectionMatch) validateCollection();
            else if(planned(park())+v[2]+parkMargin()>=27) throw new IllegalArgumentException("Departure exceeds time budget");
        }
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
    private void checkPark(Node[] nodes,double x,double y) {
        for(Node n:nodes) {
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
        for(int i=0;i<v.length;i++) b.append(i==0?"":", ").append(values.rows[i].key).append('=').append(i==1 ? side() : v[i]);
        return b.toString();
    }
}
