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

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands.Eject;
import frc.robot.commands.ShooterCommands.Shoot;
import frc.robot.commands.ShooterCommands.ShooterIdle;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOReal;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.limelight.LimelightIOReal;
import frc.robot.subsystems.shooter.*;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Shooter shooter;

  private final Limelight limelight;
  private final Intake intake;
  private final Indexer indexer;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController copilot = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL: {
        // Real robot, instantiate hardware IO implementations
        drive =
                new Drive(
                        new GyroIONavx(),
                        new ModuleIOTalonFX(0, 3),
                        new ModuleIOTalonFX(1, 4),
                        new ModuleIOTalonFX(2, 7),
                        new ModuleIOTalonFX(3, 1));
//        drive = null;
        shooter =
                new Shooter(
                        new FlywheelIOTalonFX(),
                        new FlywheelIOSparkMaxSingle(1, 13, false, 20),
                        new FlywheelIOSparkMaxSingle(1.0/10.0, 14, false, 40));
        limelight = new Limelight(new LimelightIOReal());
//        limelight = null;
        intake = new Intake(new IntakeIOReal());
//        intake = null;
        indexer = new Indexer(new IndexerIOReal());
        break;
      }

      case SIM: {

        // Sim robot, instantiate physics sim IO implementations
        drive =
                new Drive(
                        new GyroIO() {
                        },
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        new ModuleIOSim());
        shooter = null;
        limelight = null;
        intake = null;
        indexer = null;
        break;
      }

      default: {
        // Replayed robot, disable IO implementations
        drive =
                new Drive(
                        new GyroIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        });
        shooter = null;
        limelight = null;
        intake = null;
        indexer = null;
        break;
      }
    }
//     Set up named commands for PathPlanner
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    configureAutoChooser();


    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftX(),
            () -> controller.getLeftY(),
            () -> -controller.getRightX()));

    shooter.setDefaultCommand(new ShooterIdle(shooter));
//      shooter.setDefaultCommand(new ShooterCommands.ShooterTesting(shooter));
//      intake.setDefaultCommand(new IntakeCommands.IntakeTesting(intake));
    indexer.setDefaultCommand(new InstantCommand(() -> indexer.setVoltage(9), indexer));


    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    controller.leftBumper().onTrue(new InstantCommand(() -> drive.resetPose()));
//    controller
//        .b()
//        .onTrue(
//            Commands.runOnce(
//                    () ->
//                        drive.setPose(
//                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
//                    drive)
//                .ignoringDisable(true));

    controller.
        rightTrigger(0.7)
            .onTrue(new IntakeCommands.IntakeDown(intake))
            .onFalse(new IntakeCommands.IntakeUp(intake));
    controller.rightBumper().onTrue(new IntakeCommands.IntakeUp(intake));

    copilot
//    controller
            .rightTrigger(0.7)
            .whileTrue(DriveCommands.LimelightRotDrive(
              drive,
              () -> -controller.getLeftX(),
              () -> controller.getLeftY(),
              () -> -controller.getRightX(),
                    limelight))
            .whileTrue(new Shoot(shooter,limelight));

    copilot
//    controller
            .leftTrigger(0.7)
            .whileTrue(DriveCommands.LimelightRotDrive(
                    drive,
                    () -> -controller.getLeftX(),
                    () -> controller.getLeftY(),
                    () -> -controller.getRightX(),
                    limelight))
            .whileTrue(new RepeatCommand(new Shoot(shooter, limelight)));
    copilot.a().whileTrue(new Eject(shooter));

//    controller.x().whileTrue(new ShooterCommands.ShooterTesting(shooter));
  }

  private void configureAutoChooser() {
    // Set up auto routines

    // Set up FF characterization routines
//    autoChooser.addOption(
//        "Drive FF Characterization",
//        new FeedForwardCharacterization(
//            drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity));

//    autoChooser.addOption(
//            "Shooter Main FF Characterization",
//            new FeedForwardCharacterization(
//                    shooter,
//                    shooter::runMainCharacterizationVolts,
//                    shooter::getMainCharacterizationVelocity));
//    autoChooser.addOption(
//            "Shooter Secondary FF Characterization",
//            new FeedForwardCharacterization(
//                    shooter,
//                    shooter::runSecondaryCharacterizationVolts,
//                    shooter::getSecondaryCharacterizationVelocity));
    autoChooser.addOption(
            "Justin Case",
            new DriveCommands.JustinCase(drive, intake));

    autoChooser.addOption(
            "Do Nothing",
            new SequentialCommandGroup(
                    new InstantCommand(() -> drive.resetPose()),
                    new InstantCommand(() -> intake.zeroPivotAngle()),
                    new IntakeCommands.IntakeUp(intake)));
    autoChooser.addOption(
            "Shoot and Drive",
            new DriveCommands.ShootAndDrive(drive, intake, shooter, limelight, indexer)
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
//    return new SequentialCommandGroup(
//            new InstantCommand(() -> drive.resetPose()),
//            new InstantCommand(() -> intake.zeroPivotAngle()),
//            new IntakeCommands.IntakeUp(intake),
            return autoChooser.get();
//    );
  }
}
