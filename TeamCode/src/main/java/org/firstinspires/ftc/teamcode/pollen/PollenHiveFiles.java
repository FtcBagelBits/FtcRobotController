package org.firstinspires.ftc.teamcode.pollen;

import java.io.*;
import java.util.Properties;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

/** On-hub settings survive an app restart. Each start spot/alliance needs its own numbered slot. */
public final class PollenHiveFiles {
    private PollenHiveFiles() { }
    private static File file(String name) {
        File dir=new File(AppUtil.FIRST_FOLDER,"pollen-hive");
        if(!dir.isDirectory() && !dir.mkdirs()) throw new IllegalStateException("Cannot create "+dir);
        return new File(dir,name+".properties");
    }
    public static void saveCapture(int slot,String label,int pipeline,int tag,IsHiveReadyForShot.Capture capture) throws IOException {
        Properties p=new Properties(); p.setProperty("pipeline",""+pipeline); p.setProperty("tag",""+tag);
        p.setProperty("state",label); p.setProperty("units","metres, camera coordinates");
        p.setProperty("samplesCsv",capture.samples.toString());
        save("capture-"+slot+"-"+label+"-"+System.currentTimeMillis(),p);
    }
    public static Properties read(String name) throws IOException {
        Properties p=new Properties();
        try(InputStream in=new FileInputStream(file(name))) { p.load(in); }
        return p;
    }
    public static void save(String name, Properties p) throws IOException {
        File dest=file(name), temp=new File(dest.getPath()+".tmp");
        try(FileOutputStream out=new FileOutputStream(temp)) { p.store(out,"Pollen hive v1"); out.getFD().sync(); }
        if(!temp.renameTo(dest)) throw new IOException("Cannot replace "+dest);
    }
    public static IsHiveReadyForShot.Profile loadProfile(int slot) throws IOException {
        Properties p=read("profile-"+slot);
        if(!"1".equals(p.getProperty("version"))) throw new IOException("Unsupported calibration");
        return new IsHiveReadyForShot.Profile(Integer.parseInt(p.getProperty("pipeline")),
                Integer.parseInt(p.getProperty("tag")),box(p,"up"),
                "true".equals(p.getProperty("hasDown"))?box(p,"down"):null);
    }
    private static IsHiveReadyForShot.Box box(Properties p,String label) {
        double[] lo=new double[3],hi=new double[3];
        for(int i=0;i<3;i++) { lo[i]=Double.parseDouble(p.getProperty(label+"Lo"+i)); hi[i]=Double.parseDouble(p.getProperty(label+"Hi"+i)); }
        return new IsHiveReadyForShot.Box(lo,hi);
    }
    private static void box(Properties p,String label,IsHiveReadyForShot.Box b) {
        for(int i=0;i<3;i++) { p.setProperty(label+"Lo"+i,""+b.lo[i]); p.setProperty(label+"Hi"+i,""+b.hi[i]); }
    }
    public static void saveProfile(int slot,IsHiveReadyForShot.Profile c) throws IOException {
        Properties p=new Properties();
        p.setProperty("version","1"); p.setProperty("pipeline",""+c.pipeline); p.setProperty("tag",""+c.tag);
        p.setProperty("lensHeightInches","14"); p.setProperty("hasDown",""+(c.down!=null));
        box(p,"up",c.up); if(c.down!=null) box(p,"down",c.down);
        save("profile-"+slot,p);
    }
}
