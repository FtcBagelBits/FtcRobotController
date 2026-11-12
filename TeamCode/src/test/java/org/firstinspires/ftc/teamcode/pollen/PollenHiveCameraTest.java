package org.firstinspires.ftc.teamcode.pollen;
/** Runs with real org.json (Gradle test dependency), no connected camera. */
public final class PollenHiveCameraTest {
    private static int checks;
    private static final String GOOD="{\"pID\":0,\"v\":1,\"cl\":10,\"tl\":10,\"ts\":123,\"Fiducial\":[{\"fID\":34,\"t6t_cs\":[0,-0.5,1.2,0,0,0]}]}";
    private static void reject(String json,int pipeline,int tag,long age) {
        checks++;
        try { PollenHiveCamera.decode(json,pipeline,tag,age); }
        catch(Exception expected) { return; }
        throw new AssertionError("Accepted invalid JSON: "+json);
    }
    public static void main(String[] args) throws Exception {
        IsHiveReadyForShot.Frame f=PollenHiveCamera.decode(GOOD,0,34,10);
        if(!f.usable()||f.y!=-.5||f.stamp!=123) throw new AssertionError("Valid frame parsed incorrectly");
        checks++;
        reject(GOOD,1,34,10); reject(GOOD,0,35,10); reject(GOOD,0,34,151); reject(GOOD,0,34,-1);
        reject(GOOD.replace("\"cl\":10","\"cl\":190"),0,34,10);
        reject(GOOD.replace("\"cl\":10","\"cl\":-1"),0,34,0);
        reject(GOOD.replace("\"ts\":123,",""),0,34,0);
        reject(GOOD.replace("\"ts\":123","\"ts\":0"),0,34,0);
        reject(GOOD.replace("[0,-0.5,1.2,0,0,0]","[0,0,0,0,0,0]"),0,34,0);
        reject(GOOD.replace("[0,-0.5,1.2,0,0,0]","[0,-0.5,1.2]"),0,34,0);
        reject(GOOD.replace("\"t6t_cs\"","\"missing\""),0,34,0);
        reject(GOOD.replace("}]}","},{\"fID\":34,\"t6t_cs\":[0,-0.5,1.2,0,0,0]}]}"),0,34,0);
        System.out.println("PASS PollenHiveCameraTest: "+checks+" checks (real JSON parser; no hardware)");
    }
}
