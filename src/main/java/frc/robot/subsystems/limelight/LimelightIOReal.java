package frc.robot.subsystems.limelight;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.LimelightHelpers;

import java.util.ArrayList;
import java.util.Optional;

public class LimelightIOReal implements LimelightIO{

    LimelightHelpers.LimelightResults limelightResults;
    public LimelightIOReal(){
        LimelightHelpers.setPipelineIndex("",9);
        LimelightHelpers.setLEDMode_PipelineControl("");
    }

    @Override
    public void updateInputs(LimelightIOInputs inputs) {
        limelightResults = LimelightHelpers.getLatestResults("");
        inputs.fullDump = limelightResults.toString();
        inputs.validTarget = limelightResults.targetingResults.valid;
        inputs.captureLatencyMs = limelightResults.targetingResults.latency_capture;
        inputs.pipelineLatencyMs = limelightResults.targetingResults.latency_pipeline;
        inputs.parseLatencyMs = limelightResults.targetingResults.latency_jsonParse;
        inputs.totalLatencyMs = inputs.captureLatencyMs + inputs.pipelineLatencyMs + inputs.parseLatencyMs;
        inputs.timestampMs = limelightResults.targetingResults.timestamp_RIOFPGA_capture;
        inputs.pipeline = limelightResults.targetingResults.pipelineID;

        inputs.allTargets = new double[limelightResults.targetingResults.targets_Detector.length][];
        ArrayList<double[]> validTargets = new ArrayList<>(inputs.allTargets.length);
        for (int i = 0; i < inputs.allTargets.length; i += 1) {
            LimelightHelpers.LimelightTarget_Detector target = limelightResults.targetingResults.targets_Detector[i];
            double[] temp = new double[]{target.classID, target.confidence, target.ta, target.tx, target.ty};
            inputs.allTargets[i] = temp;
            if(target.classID == (DriverStation.getAlliance().get() == DriverStation.Alliance.Red?1.0:0)){
                validTargets.add(temp);
            }
        }
        inputs.validTargets = (double[][]) validTargets.toArray();
    }
}
