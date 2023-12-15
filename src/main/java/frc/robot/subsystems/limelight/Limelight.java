package frc.robot.subsystems.limelight;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;

public class Limelight extends SubsystemBase {
    private final LimelightIO io;
    private final LimelightIOInputsAutoLogged inputs = new LimelightIOInputsAutoLogged();
    private double[] target;

    private final double mountingAngleDegrees = 10.0;
    private final double limelightLensHeightInches = 20.0;//TODO Find this
    private final double goalHeightInches = 25.0;//TODO Find this
    private static  double HORIZONTAL_FOV = Math.toRadians(63.3);
    private static  double VERTICAL_FOV = Math.toRadians(49.7);

    public Limelight(LimelightIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Limelight", inputs);


        if(getTv()) target = bestTargetCentral();

        Logger.getInstance().recordOutput("Limelight/targetDistance", getDistance());
        Logger.getInstance().recordOutput("Limelight/targetX", getTx());
        Logger.getInstance().recordOutput("Limelight/targetY", getTy());
        Logger.getInstance().recordOutput("Limelight/validTargetsLength", io.getValidTargets().size());
        Logger.getInstance().recordOutput("Limelight/allTargetsLength", io.getAllTargets().length);
    }

    public double getTa() {
        return target[2];
    }

    public double getTx() {
        if(getTv()) return target[3];
        return 0;
    }
    public double getTx(double[] target){
        return target[3];
    }

    public double getTy() {
        if(getTv()) return target[4];
        return 0;
    }

    public boolean getTv() {
        return inputs.validTarget && io.getValidTargets().size() > 0;
    }

    public double getDistance(){
        if(getTv()) return (goalHeightInches - limelightLensHeightInches)/Math.tan(Math.toRadians(getTy() + mountingAngleDegrees));
        return 0;
    }
    public double getDistance(double[] target){
        if(getTv()) return (goalHeightInches - limelightLensHeightInches)/Math.tan(Math.toRadians(target[4] + mountingAngleDegrees));
        return 0;
    }

    private double[] bestTargetMinDist(){
        if (!getTv()) return null ;
        ArrayList<double[]> targets = io.getValidTargets();
        double[] minTarget = targets.get(0);
        double min = getDistance(minTarget);
        for (int i = 1; i < targets.size(); i++) {
            if(getDistance(targets.get(i)) < min){
                min = getDistance(targets.get(i));
                minTarget = targets.get(i);
            }
        }
        return minTarget;
    }

    private double[] bestTargetCentral(){
        if (!getTv()) return null;
        ArrayList<double[]> targets = io.getValidTargets();
        double[] minTarget = targets.get(0);
        double min = getTx(targets.get(0));
        for (int i = 1; i < targets.size(); i++) {
            if(getTx(targets.get(i)) < min){
                min = getTx(targets.get(i));
                minTarget = targets.get(i);
            }
        }
        return minTarget;

    }

    public boolean aimed(){
        return getTx() < 1;
    }

    public double getHorizontalFov() {
        return HORIZONTAL_FOV;
    }
}
