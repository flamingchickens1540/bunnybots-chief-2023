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

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.limelight.Limelight;

import java.util.function.DoubleSupplier;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static PIDController pid = new PIDController(1,0,0);

  private DriveCommands() {}

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(new Translation2d(linearMagnitude, 0.0), new Rotation2d()))
                  .getTranslation();

          // Convert to field relative speeds & send command
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec(),
                  drive.getRotation()));
        },
        drive);
  }

    public static Command LimelightRotDrive(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            Limelight limelight) {
        return Commands.run(
                () -> {

                    // Apply deadband
                    double linearMagnitude =
                            MathUtil.applyDeadband(
                                    Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
                    Rotation2d linearDirection =
                            new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
                    double omega = MathUtil.applyDeadband(limelight.getTx() * limelight.getHorizontalFov()/(2*100), 0.01);
                    // Square values
                    linearMagnitude = linearMagnitude * linearMagnitude;
//                    omega = Math.copySign(omega * omega, omega);

                    // Calcaulate new linear velocity
                    Translation2d linearVelocity =
                            new Pose2d(new Translation2d(), linearDirection)
                                    .transformBy(new Transform2d(new Translation2d(linearMagnitude, 0.0), new Rotation2d()))
                                    .getTranslation();

                    // Convert to field relative speeds & send command
                    drive.runVelocity(
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                    linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                                    linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                                    pid.calculate(omega, 0) * drive.getMaxAngularSpeedRadPerSec(),
//                                    omega * drive.getMaxAngularSpeedRadPerSec(),

                                    drive.getRotation()));
                },
                drive);
    }

    public static class JustinCase extends SequentialCommandGroup {
        public JustinCase(Drive drive){
            addCommands(
                    new ParallelRaceGroup(
                            new WaitCommand(1),
                            joystickDrive(drive, () -> 0.0, () -> 0.5, () -> 0.0)
                    ),
                    joystickDrive(drive, () -> 0.0, () -> 0.0, () -> 0.0)
            );
        }
    }
}
