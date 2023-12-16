package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs {
        public double velocityRadPerSec;
        public double appliedVolts;
        public double currentAmps;
    }

    public default void updateInputs(IndexerIOInputs inputs) {}
    public default void setVoltage(double volts){}
    public default void stop(){}
}