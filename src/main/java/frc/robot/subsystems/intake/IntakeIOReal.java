package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class IntakeIOReal implements IntakeIO {
  private final TalonFX roller = new TalonFX(Constants.Intake.rollerID);
  private final CANSparkMax leadSpark =
      new CANSparkMax(Constants.Intake.leadPivotID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax followSpark =
      new CANSparkMax(Constants.Intake.followerPivotID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final SparkMaxPIDController leadSparkPID = leadSpark.getPIDController();
  private final SparkMaxPIDController followSparkPID = followSpark.getPIDController();

  private final StatusSignal<Double> rollerDutyCycle = roller.getDutyCycle();
  private final StatusSignal<Double> rollerSupplyVoltage =  roller.getSupplyVoltage();
  private final StatusSignal<Double> rollerCurrent = roller.getStatorCurrent();

  public IntakeIOReal() {
    //        roller.

    leadSpark.restoreFactoryDefaults();
    followSpark.restoreFactoryDefaults();

    leadSpark.getEncoder().setPosition(0);
    followSpark.getEncoder().setPosition(0);
    leadSpark.setInverted(false);
    followSpark.setInverted(true);

    leadSpark.enableVoltageCompensation(12.0);
    leadSpark.setSmartCurrentLimit(30);
    followSpark.enableVoltageCompensation(12.0);
    followSpark.setSmartCurrentLimit(30);

    leadSpark.burnFlash();
    followSpark.burnFlash();

    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 30.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    roller.getConfigurator().apply(config);
    roller.setInverted(true);

    rollerDutyCycle.setUpdateFrequency(50.0);
    rollerSupplyVoltage.setUpdateFrequency(50.0);
    rollerCurrent.setUpdateFrequency(50.0);
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    inputs.pivotCurrent =
        new double[] {leadSpark.getOutputCurrent(), followSpark.getOutputCurrent()};
    inputs.pivotVoltage = leadSpark.getBusVoltage() * leadSpark.getAppliedOutput();
    inputs.pivotPosition = new double[]{leadSpark.getEncoder().getPosition(), followSpark.getEncoder().getPosition()};
    inputs.pivotVelocity = leadSpark.getEncoder().getVelocity();

    rollerDutyCycle.refresh();
    rollerSupplyVoltage.refresh();
    rollerCurrent.refresh();

    inputs.rollerVoltage = rollerDutyCycle.getValue() * rollerSupplyVoltage.getValue();
    inputs.rollerCurrent = rollerCurrent.getValue();
  }

  @Override
  public void setPivotVoltage(double volts) {
    leadSpark.setVoltage(volts);
  }

  @Override
  public void setRollerVoltage(double volts) {
    roller.setControl(new VoltageOut(volts));
    //        roller.setVoltage(volts);
  }

  @Override
  public void setPivotBreakMode(boolean breakMode) {
    CANSparkMax.IdleMode idleMode = CANSparkMax.IdleMode.fromId(breakMode ? 1 : 0);
    leadSpark.setIdleMode(idleMode);
    followSpark.setIdleMode(idleMode);
  }

  @Override
  public void configureLeadPID(double kP, double kI, double kD) {
    leadSparkPID.setP(kP, 0);
    leadSparkPID.setI(kI, 0);
    leadSparkPID.setD(kD, 0);
    leadSparkPID.setFF(0, 0);
  }

  @Override
  public void configureSecondPID(double kP, double kI, double kD){
    followSparkPID.setP(kP, 0);
    followSparkPID.setI(kI, 0);
    followSparkPID.setD(kD, 0);
    followSparkPID.setFF(0, 0);
  }

  @Override
  public void setPivotPosition(Rotation2d position) {
    System.out.println("Set Pivot Position IO");
    leadSparkPID.setReference(position.getRotations(), CANSparkMax.ControlType.kPosition);
    followSparkPID.setReference(position.getRotations(), CANSparkMax.ControlType.kPosition);;
  }

  @Override
  public void zeroPivotPosition() {
    leadSpark.getEncoder().setPosition(0);
    followSpark.getEncoder().setPosition(0);
  }
}
