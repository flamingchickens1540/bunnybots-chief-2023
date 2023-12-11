package frc.robot.subsystems.limelight;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

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
        if(getTv()) target = bestTargetMinDist();

        Logger.getInstance().recordOutput("Limelight/targetDistance", getDistance());
    }

    public double getTa() {
        return target[2];
    }

    public double getTx() {
        if(getTv()) return target[3];
        return 0;
    }

    public double getTy() {
        if(getTv()) return target[4];
        return 0;
    }

    public boolean getTv() {
        return inputs.validTarget;
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
        if (!getTv()) return null;
        double[] minTarget = io.validTargets[0];
        double min = getDistance(minTarget);
        for (int i = 1; i < io.validTargets.length; i++) {
            if(getDistance(io.validTargets[i]) < min){
                min = getDistance(io.validTargets[i]);
                minTarget = io.validTargets[i];
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
