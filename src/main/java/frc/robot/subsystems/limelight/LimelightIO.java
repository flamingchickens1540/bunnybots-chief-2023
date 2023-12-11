package frc.robot.subsystems.limelight;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface LimelightIO {
    public double[][] allTargets = {};
    public double[][] validTargets = {};

    @AutoLog
    public static class LimelightIOInputs{
        public String fullDump = "";
        public boolean validTarget = false;

        public double captureLatencyMs = 0.0;
        public double pipelineLatencyMs = 0.0;
        public double parseLatencyMs = 0.0;
        public double totalLatencyMs = 0.0;
        public double timestampMs = 0.0;
        public double pipeline = 0;

//        public double[][] allTargets = {};
//        public double[][] validTargets = {};
    }

    public default void updateInputs(LimelightIOInputs inputs) {}
}
