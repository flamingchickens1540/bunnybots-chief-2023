package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs {
      public double velocityRotationsPerSecond;
    }
  
    public default void updateInputs(IndexerIOInputs inputs) {}
    public default void setPercent(double input) {}
}
