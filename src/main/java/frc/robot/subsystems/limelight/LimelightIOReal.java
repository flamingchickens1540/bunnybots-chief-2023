package frc.robot.subsystems.limelight;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.LimelightHelpers;

import java.util.ArrayList;
import java.util.Optional;

public class LimelightIOReal implements LimelightIO{

    public double[][] allTargets = {};
    public ArrayList<double[]> validTargets = new ArrayList<>();

    LimelightHelpers.LimelightResults limelightResults;
    public LimelightIOReal(){
        LimelightHelpers.setPipelineIndex("limelight-rear",9);
        LimelightHelpers.setLEDMode_PipelineControl("");
    }

    @Override
    public void updateInputs(LimelightIOInputs inputs) {
        validTargets.clear();
        limelightResults = LimelightHelpers.getLatestResults("limelight-rear");
        inputs.fullDump = "bruh";
        inputs.validTarget = limelightResults.targetingResults.valid;
        inputs.captureLatencyMs = limelightResults.targetingResults.latency_capture;
        inputs.pipelineLatencyMs = limelightResults.targetingResults.latency_pipeline;
        inputs.parseLatencyMs = limelightResults.targetingResults.latency_jsonParse;
        inputs.totalLatencyMs = inputs.captureLatencyMs + inputs.pipelineLatencyMs + inputs.parseLatencyMs;
        inputs.timestampMs = limelightResults.targetingResults.timestamp_RIOFPGA_capture;
        inputs.pipeline = limelightResults.targetingResults.pipelineID;

        allTargets = new double[limelightResults.targetingResults.targets_Detector.length][];
        ArrayList<double[]> validTargets1 = new ArrayList<>(allTargets.length);
        for (int i = 0; i < allTargets.length; i += 1) {
            LimelightHelpers.LimelightTarget_Detector target = limelightResults.targetingResults.targets_Detector[i];
            double[] temp = new double[]{target.classID, target.confidence, target.ta, target.tx, target.ty};
            allTargets[i] = temp;
            if(target.classID == (DriverStation.getAlliance() == DriverStation.Alliance.Blue?1.0:0)){
                validTargets1.add(temp);
            }
        }
        validTargets = validTargets1;
    }

    public double[][] getAllTargets(){
        return allTargets;
    }

    @Override
    public ArrayList<double[]> getValidTargets() {
        return validTargets;
    }
}
