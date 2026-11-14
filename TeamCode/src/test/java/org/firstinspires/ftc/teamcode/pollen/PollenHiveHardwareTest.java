package org.firstinspires.ftc.teamcode.pollen;

import com.qualcomm.robotcore.hardware.*;
import java.lang.reflect.Proxy;

/** SDK-interface tests for the map guard; no Android motor controller is instantiated. */
public final class PollenHiveHardwareTest {
    private static int checks;
    private static void check(boolean ok,String why) { checks++; if(!ok) throw new AssertionError(why); }
    private static DcMotorController controller() {
        return (DcMotorController)Proxy.newProxyInstance(DcMotorController.class.getClassLoader(),new Class<?>[]{DcMotorController.class},
                (p,m,a)->m.getName().equals("getConnectionInfo")?"test controller":null);
    }
    private static DcMotor motor(DcMotorController controller,int port) {
        return (DcMotor)Proxy.newProxyInstance(DcMotor.class.getClassLoader(),new Class<?>[]{DcMotor.class},(p,m,a)-> {
            switch(m.getName()) {
                case "getController": return controller;
                case "getPortNumber": return port;
                case "getPower": return 0.0;
                case "getCurrentPosition": return 123;
                default: return null;
            }
        });
    }
    private static boolean rejected(DcMotor a,DcMotor b) {
        try { PollenHiveHardware.requireDistinct(new String[]{"drive","launcher"},new DcMotor[]{a,b}); return false; }
        catch(IllegalStateException ex) { return ex.getMessage().contains("drive and launcher"); }
    }
    public static void main(String[] args) {
        DcMotorController a=controller(), b=controller(); DcMotor m=motor(a,0);
        check(rejected(m,m),"Same motor object blocked");
        check(rejected(m,motor(a,0)),"Different objects on same controller port blocked");
        check(!rejected(m,motor(a,1)),"Different ports allowed");
        check(!rejected(m,motor(b,0)),"Same port on different controllers allowed");
        check(PollenHiveHardware.sample(m).contains("encoder=123"),"Encoder and SDK power available without DcMotorEx");
        check(Double.isNaN(PollenHiveHardware.launcherCurrent(null)),"Unavailable optional current does not throw");
        System.out.println("PASS PollenHiveHardwareTest: "+checks+" checks");
    }
}
