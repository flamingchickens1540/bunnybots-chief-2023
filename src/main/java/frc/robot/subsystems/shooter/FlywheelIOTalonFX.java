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

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class FlywheelIOTalonFX implements FlywheelIO {
  private static final double GEAR_RATIO = 22 / 16;

  private final TalonFX leader = new TalonFX(11);
  private final TalonFX follower = new TalonFX(12);

  private final StatusSignal<Double> leaderPosition = leader.getPosition();
  private final StatusSignal<Double> leaderVelocity = leader.getVelocity();
  private final StatusSignal<Double> leaderDutyCycle = leader.getDutyCycle();
  private final StatusSignal<Double> leaderSupplyVoltage =  leader.getSupplyVoltage();
  private final StatusSignal<Double> leaderCurrent = leader.getStatorCurrent();
  private final StatusSignal<Double> followerCurrent = follower.getStatorCurrent();

  public FlywheelIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 40.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    leader.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);
    follower.setControl(new Follower(leader.getDeviceID(), true));

    leaderPosition.setUpdateFrequency(50.0);
    leaderVelocity.setUpdateFrequency(50.0);
    leaderSupplyVoltage.setUpdateFrequency(50.0);
    leaderDutyCycle.setUpdateFrequency(50.0);
    leaderCurrent.setUpdateFrequency(50.0);
    followerCurrent.setUpdateFrequency(50.0);
//    leader.optimizeBusUtilization();
//    follower.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    leaderPosition.refresh();
    leaderVelocity.refresh();
    leaderSupplyVoltage.refresh();
    leaderDutyCycle.refresh();
    leaderCurrent.refresh();
    followerCurrent.refresh();

    inputs.positionRad = Units.rotationsToRadians(leaderPosition.getValue()) / GEAR_RATIO;
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(leaderVelocity.getValue()) / GEAR_RATIO;
    inputs.appliedVolts = leaderSupplyVoltage.getValue() * leaderDutyCycle.getValue();
    inputs.currentAmps =
        new double[] {leaderCurrent.getValue(), followerCurrent.getValue()};
  }

  @Override
  public void setVoltage(double volts) {
    leader.setControl(new VoltageOut(volts));
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    leader.setControl(
        new VelocityVoltage(
            Units.radiansToRotations(velocityRadPerSec), true, ffVolts, 0, false));
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    Slot0Configs config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    leader.getConfigurator().apply(config);
  }
}
