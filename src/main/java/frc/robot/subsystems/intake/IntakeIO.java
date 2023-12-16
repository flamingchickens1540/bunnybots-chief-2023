package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  class IntakeInputs {
    public double pivotVoltage = 0.0;
    public double[] pivotCurrent = new double[] {};
    public double[] pivotPosition = new double[]{};
    public double pivotVelocity = 0.0;
    public double rollerVoltage = 0.0;
    public double rollerCurrent = 0.0;
  }

  public default void updateInputs(IntakeInputs inputs) {}

  public default void setPivotVoltage(double volts) {}

  public default void setRollerVoltage(double volts) {}

  public default void setPivotBreakMode(boolean breakMode) {}

  public default void setPivotPosition(Rotation2d position){}
  public default void zeroPivotPosition(){}

  public default void configureLeadPID(double kP, double kI, double kD){}
  public default void configureSecondPID(double kP, double kI, double kD){}
}
