package org.firstinspires.ftc.teamcode.pollen;

import java.io.*;
import java.nio.file.*;

/** Replays the observed risks; does not simulate flywheel physics or validate wiring. */
public final class PollenHiveDiagnosticsTest {
    private static int checks;
    private static void check(boolean ok,String why) { checks++; if(!ok) throw new AssertionError(why); }
    private static PollenHiveAuto auto(PollenHiveRoute c) {
        PollenHiveAuto a=new PollenHiveAuto(c,new IsHiveReadyForShot(PollenHiveTest.M)); a.start(0); return a;
    }
    private static void unexpectedMotion() {
        for(double[] pose:new double[][]{{2.01,0,0},{0,0,Math.toRadians(-10.1)}}) {
            PollenHiveAuto a=auto(new PollenHiveRoute(false));
            a.tick(.02,null,0,0,0,0,true);
            PollenHiveAuto.Output o=a.tick(.04,null,pose[0],pose[1],pose[2],1180,true);
            check(o.finished&&o.reason.contains("Unexpected motion"),"Unexpected chassis motion stops, not parks");
            check(o.forward==0&&o.strafe==0&&o.turn==0&&o.launcher==0&&o.intake==0&&o.agitator==0,"All stopped outputs zero");
            check(o.decision.contains("evaluated=FIRST"),"Aborted decision retains original phase");
            check(a.tick(.06,null,0,0,0,1180,true).finished,"Abort latched even if pose recovers");
        }
        PollenHiveAuto a=auto(new PollenHiveRoute(false));
        check(!a.tick(.02,null,.1,0,Math.toRadians(1),0,true).finished,"Small placement noise does not abort");
        PollenHiveRoute delayed=new PollenHiveRoute(false); delayed.v[2]=1;
        check(auto(delayed).tick(.02,null,3,0,0,0,true).finished,"Delay also requires stationary chassis");
    }
    private static void coast() {
        PollenHiveRoute c=new PollenHiveRoute(false); c.v[0]=1;
        PollenHiveAuto a=auto(c); PollenHiveAuto.Output o=null;
        for(int i=1;i<=110;i++) {
            o=a.tick(i*.02,null,0,0,0,101,true);
            if(o.phase.startsWith("PARK")) break;
        }
        check(o.phase.startsWith("PARK")&&o.reason.contains("flywheel did not stop"),"Measured 101 fails coast timeout");
        check(o.decision.contains("evaluated=COAST")&&o.decision.contains("coast_speed_ok=false"),"Timeout logs failing gate before transition");
        check(o.decision.contains("garden_need_s=")&&o.decision.contains("remaining_s="),"Budget evidence recorded");
        a=auto(c);
        for(int i=1;i<=40;i++) { o=a.tick(i*.02,null,0,0,0,100,true); if(o.phase.startsWith("GARDEN")) break; }
        check(o.phase.startsWith("GARDEN"),"100 ticks/s allowed after minimum coast time");
        check(o.decision.contains("coast_min_elapsed=true")&&o.decision.contains("coast_speed_ok=true"),"Passing gates logged");
        o=a.stop("Driver STOP");
        check(o.finished&&o.reason.equals("Driver STOP"),"Intentional STOP distinct from fault");
    }
    private static void trace() throws Exception {
        StringWriter out=new StringWriter(); PollenHiveTrace trace=new PollenHiveTrace(out);
        trace.row(0,true,"reason","a,b\"c",1.5);
        check(out.toString().equals("\"reason\",\"a,b\"\"c\",\"1.5000\"\n"),"CSV escaping and stable decimal format");
        trace.close(); int size=out.toString().length(); trace.row(1,true,"ignored");
        check(out.toString().length()==size,"No writes after close");
        Writer broken=new Writer() {
            public void write(char[] c,int o,int n) throws IOException { throw new IOException("disk unavailable"); }
            public void flush() {} public void close() {}
        };
        trace=new PollenHiveTrace(broken); trace.row(0,true,"test");
        check(trace.status.contains("failed"),"Write failure reported without throwing into control loop");
        out=new StringWriter(); trace=new PollenHiveTrace(out);
        char[] big=new char[100000]; java.util.Arrays.fill(big,'x');
        for(int i=0;i<30;i++) trace.row(i,false,new String(big));
        check(trace.status.contains("limit")&&out.toString().length()<=PollenHiveTrace.RUN_LIMIT,"Per-run trace bounded");
        Path dir=Files.createTempDirectory("hive-trace-test");
        try {
            File full=dir.resolve("old.csv").toFile();
            try(RandomAccessFile f=new RandomAccessFile(full,"rw")) { f.setLength(PollenHiveTrace.TOTAL_LIMIT); }
            boolean blocked=false;
            try { new PollenHiveTrace(dir.toFile(),"new"); } catch(IOException ex) { blocked=ex.getMessage().contains("budget"); }
            check(blocked,"Total trace budget prevents creating another run");
            check(full.length()==PollenHiveTrace.TOTAL_LIMIT,"Budget check preserves existing evidence");
            Files.delete(full.toPath());
        } finally { Files.delete(dir); }
    }
    public static void main(String[] args) throws Exception {
        unexpectedMotion(); coast(); trace();
        System.out.println("PASS PollenHiveDiagnosticsTest: "+checks+" checks");
    }
}
