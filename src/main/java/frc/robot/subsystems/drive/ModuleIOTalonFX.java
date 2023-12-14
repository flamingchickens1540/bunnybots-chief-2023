// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using an analog encoder, copy from "ModuleIOSparkMax")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOTalonFX implements ModuleIO {
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder cancoder;

  private final StatusSignal<Double> drivePosition;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveDutyCycle;
  private final StatusSignal<Double> driveSupplyVoltage;
  private final StatusSignal<Double> driveCurrent;

  private final StatusSignal<Double> turnAbsolutePosition;
  private final StatusSignal<Double> turnPosition;
  private final StatusSignal<Double> turnVelocity;
  private final StatusSignal<Double> turnDutyCycle;
  private final StatusSignal<Double> turnSupplyVoltage;
  private final StatusSignal<Double> turnCurrent;

  // Gear ratios for SDS MK4i L2, adjust as necessary
  private final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
  private final double TURN_GEAR_RATIO = 150.0 / 7.0;

  private final boolean isTurnMotorInverted = true;
  private final Rotation2d absoluteEncoderOffset;

  public ModuleIOTalonFX(int index, int moduleNumber) {
      driveTalon = new TalonFX(moduleNumber + 30, "swerve");
      turnTalon = new TalonFX(moduleNumber + 20, "swerve");
      cancoder = new CANcoder(moduleNumber + 10, "swerve");
      absoluteEncoderOffset = Rotation2d.fromDegrees((Constants.Drivetrain.cornerOffsets[index] + Constants.Drivetrain.offsets[moduleNumber-1]) % 360); // MUST BE CALIBRATED

    var driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveTalon.getConfigurator().apply(driveConfig);
    setDriveBrakeMode(true);

    var turnConfig = new TalonFXConfiguration();
    turnConfig.CurrentLimits.StatorCurrentLimit = 30.0;
    turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    turnTalon.getConfigurator().apply(turnConfig);
    setTurnBrakeMode(true);

    cancoder.getConfigurator().apply(new CANcoderConfiguration());

    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveDutyCycle = driveTalon.getDutyCycle();
    driveSupplyVoltage = driveTalon.getSupplyVoltage();
    driveCurrent = driveTalon.getStatorCurrent();

    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();
    turnVelocity = turnTalon.getVelocity();
    turnDutyCycle = turnTalon.getDutyCycle();
    turnSupplyVoltage = turnTalon.getSupplyVoltage();
    turnCurrent = turnTalon.getStatorCurrent();


 // Required for odometry, use faster rate
    drivePosition.setUpdateFrequency(100.0);
    turnPosition.setUpdateFrequency(100.0);

    driveVelocity.setUpdateFrequency(50.0);
    driveDutyCycle.setUpdateFrequency(50.0);
    driveSupplyVoltage.setUpdateFrequency(50.0);
    driveCurrent.setUpdateFrequency(50.0);
    turnAbsolutePosition.setUpdateFrequency(50.0);
    turnVelocity.setUpdateFrequency(50.0);
    turnDutyCycle.setUpdateFrequency(50.0);
    turnSupplyVoltage.setUpdateFrequency(50.0);
    turnCurrent.setUpdateFrequency(50.0);
//    driveTalon.optimizeBusUtilization();
//    turnTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    drivePosition.refresh();
    driveVelocity.refresh();
    driveDutyCycle.refresh();
    driveSupplyVoltage.refresh();
    driveCurrent.refresh();
    turnAbsolutePosition.refresh();
    turnPosition.refresh();
    turnVelocity.refresh();
    turnDutyCycle.refresh();
    turnSupplyVoltage.refresh();
    turnCurrent.refresh();

    inputs.drivePositionRad =
        Units.rotationsToRadians(drivePosition.getValue()) / DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec =
        Units.rotationsToRadians(driveVelocity.getValue()) / DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveSupplyVoltage.getValue() * driveDutyCycle.getValue();
    inputs.driveCurrentAmps = new double[] {driveCurrent.getValue()};

    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsolutePosition.getValue())
            .minus(absoluteEncoderOffset).getDegrees();
    inputs.turnPosition =
        Rotation2d.fromRotations(turnPosition.getValue() / TURN_GEAR_RATIO).getDegrees();
    inputs.turnVelocityRadPerSec =
        Units.rotationsToRadians(turnVelocity.getValue()) / TURN_GEAR_RATIO;
    inputs.turnAppliedVolts = turnSupplyVoltage.getValue() * turnDutyCycle.getValue();
    inputs.turnCurrentAmps = new double[] {turnCurrent.getValue()};
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted = InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveTalon.getConfigurator().apply(config);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted =
        isTurnMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    turnTalon.getConfigurator().apply(config);
  }
}
