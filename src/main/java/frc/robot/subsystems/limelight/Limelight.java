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
        target = bestTargetMinDist();

        Logger.getInstance().recordOutput("Limelight/targetDistance", getDistance());
    }

    public double getTa() {
        return target[2];
    }

    public double getTx() {
        return target[3];
    }

    public double getTy() {
        return target[4];
    }

    public boolean getTv() {
        return inputs.validTarget;
    }

    public double getDistance(){
        return (goalHeightInches - limelightLensHeightInches)/Math.tan(Math.toRadians(getTy() + mountingAngleDegrees));
    }
    public double getDistance(double[] target){
        return (goalHeightInches - limelightLensHeightInches)/Math.tan(Math.toRadians(target[4] + mountingAngleDegrees));
    }

    private double[] bestTargetMinDist(){
        if (!inputs.validTarget) return null;
        double[] minTarget = inputs.validTargets[0];
        double min = getDistance(minTarget);
        for (int i = 1; i < inputs.validTargets.length; i++) {
            if(getDistance(inputs.validTargets[i]) < min){
                min = getDistance(inputs.validTargets[i]);
                minTarget = inputs.validTargets[i];
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
