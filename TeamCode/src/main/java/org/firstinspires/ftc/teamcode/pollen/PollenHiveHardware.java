package org.firstinspires.ftc.teamcode.pollen;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/** Configuration evidence, not proof of which physical motor a cable reaches. */
final class PollenHiveHardware {
    static void requireDistinct(String[] names, DcMotor[] motors) {
        for (int i=0;i<motors.length;i++) for (int j=0;j<i;j++) {
            DcMotor a=motors[i], b=motors[j];
            if (a==b || (a.getController()==b.getController() && a.getPortNumber()==b.getPortNumber()))
                throw new IllegalStateException("Motor port shared: "+names[j]+" and "+names[i]);
        }
    }
    static String identity(String name, DcMotor m) {
        if(m==null) return name+"=UNMAPPED";
        try { return name+" controller="+m.getController().getConnectionInfo()+" port="+m.getPortNumber()
                +" connection="+m.getConnectionInfo()+" direction="+m.getDirection()+" mode="+m.getMode()
                +" zero="+m.getZeroPowerBehavior()+" type="+m.getMotorType().getName(); }
        catch(RuntimeException ex) { return name+" identity error="+ex; }
    }
    // SDK power is a controller readback, not an independent electrical measurement.
    static String sample(DcMotor m) {
        if(m==null) return "unmapped";
        try { return "sdk_power="+m.getPower()+" encoder="+m.getCurrentPosition()
                +" velocity="+(m instanceof DcMotorEx?((DcMotorEx)m).getVelocity():Double.NaN); }
        catch(RuntimeException ex) { return "sample_error="+ex; }
    }
    static double velocity(DcMotorEx m) {
        try { return m.getVelocity(); } catch(RuntimeException ex) { return Double.NaN; }
    }
    static double launcherCurrent(DcMotorEx m) {
        try { return m.getCurrent(CurrentUnit.AMPS); } catch(RuntimeException ex) { return Double.NaN; }
    }
}
