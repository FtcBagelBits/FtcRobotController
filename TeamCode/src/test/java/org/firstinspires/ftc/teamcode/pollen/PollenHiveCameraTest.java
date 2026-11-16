package org.firstinspires.ftc.teamcode.pollen;
/** Runs with real org.json (Gradle test dependency), no connected camera. */
public final class PollenHiveCameraTest {
    private static int checks;
    private static final String GOOD="{\"pID\":0,\"v\":1,\"cl\":10,\"tl\":10,\"ts\":123,\"Fiducial\":["
        +"{\"fID\":34,\"t6t_cs\":[0,-0.5,1.2,0,0,0]},{\"fID\":35,\"t6t_cs\":[0.1,-0.5,1.2,0,0,0]},"
        +"{\"fID\":7,\"t6t_cs\":[0,0,0,0,0,0]}]}";
    private static void check(boolean ok,String why) { checks++; if(!ok) throw new AssertionError(why); }
    private static void reject(String json,int pipeline,long age) {
        checks++;
        try { PollenHiveCamera.decode(json,pipeline,age); }
        catch(Exception expected) { return; }
        throw new AssertionError("Accepted invalid JSON: "+json);
    }
    public static void main(String[] args) throws Exception {
        IsHiveReadyForShot.Frame f=PollenHiveCamera.decode(GOOD,0,10);
        check(f.usable()&&f.stamp==123&&f.tags.length==2&&f.tags[0].id==34&&f.tags[1].x==.1&&f.tags[0].y==-.5,
            "Valid frame parsed; non-hive tag 7 ignored");
        f=PollenHiveCamera.decode("{\"pID\":0,\"v\":0,\"cl\":10,\"tl\":10,\"ts\":124}",0,10);
        check(f.usable()&&f.tags.length==0,"Fresh frame with no targets = no tags (not an error)");
        reject(GOOD,1,10); reject(GOOD,0,151); reject(GOOD,0,-1);
        reject(GOOD.replace("\"cl\":10","\"cl\":190"),0,10);
        reject(GOOD.replace("\"cl\":10","\"cl\":-1"),0,0);
        reject(GOOD.replace("\"ts\":123,",""),0,0);
        reject(GOOD.replace("\"ts\":123","\"ts\":0"),0,0);
        reject(GOOD.replace("[0,-0.5,1.2,0,0,0]","[0,0,0,0,0,0]"),0,0);
        reject(GOOD.replace("[0,-0.5,1.2,0,0,0]","[0,-0.5,1.2]"),0,0);
        reject(GOOD.replace("{\"fID\":34,\"t6t_cs\"","{\"fID\":34,\"missing\""),0,0);
        reject(GOOD.replace("\"fID\":35","\"fID\":34"),0,0);
        System.out.println("PASS PollenHiveCameraTest: "+checks+" checks (real JSON parser; no hardware)");
    }
}
