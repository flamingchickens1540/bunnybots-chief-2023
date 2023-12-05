package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    private double tv, tx, ty, ta;

    private final double mountingAngleDegrees = 10.0;
    private final double limelightLensHeightInches = 20.0;//TODO Find this
    private final double goalHeightInches = 25.0;//TODO Find this
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    //private final AHRS navx;
    public final String name;
    private double[] data;
//    private final PoseZeroFilter zeroFilter = new PoseZeroFilter(50, 48);
//    private final PoseMedianFilter medianFilter = new PoseMedianFilter(10);
    private double latency;
    private static  double HORIZONTAL_FOV = Math.toRadians(63.3);
    private static  double VERTICAL_FOV = Math.toRadians(49.7);

    public Limelight(String name) {
        this.name = name;

    }

    public void update(){
        tv = table.getEntry("tv").getDouble(0);
        tx = table.getEntry("tx").getDouble(0);
        ty = table.getEntry("ty").getDouble(0);
        ta = table.getEntry("ta").getDouble(0);
    }

    public double getTa() {
        return ta;
    }

    public double getTx() {
        return tx;
    }

    public double getTy() {
        return ty;
    }

    public double getTv() {
        return tv;
    }

    public double getDistance(){
        return (goalHeightInches - limelightLensHeightInches)/Math.tan(Math.toRadians(ty + mountingAngleDegrees));
    }

    public boolean aimed(){
        return tx < 1;
    }

    public double getHorizontalFov() {
        return HORIZONTAL_FOV;
    }
}
