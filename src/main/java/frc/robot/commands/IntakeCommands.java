package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class IntakeCommands {
    public static class IntakeTesting extends CommandBase{
        LoggedDashboardNumber intakeAngle = new LoggedDashboardNumber("IntakeAngle", 0);
        LoggedDashboardNumber rollerVolts = new LoggedDashboardNumber("RollerVolts", 0);
        Intake intake;
        public IntakeTesting(Intake intake){
            this.intake = intake;
            addRequirements(intake);
        }

        @Override
        public void execute() {
            intake.setPivotAngle(Rotation2d.fromRotations(intakeAngle.get()));
            intake.rollerVoltage(rollerVolts.get());
        }

    }

    public static class IntakeUp extends CommandBase{
        Intake intake;
        public IntakeUp(Intake intake){
            this.intake = intake;
            addRequirements(intake);
        }

        @Override
        public void initialize() {
            intake.setPivotAngle(Rotation2d.fromRotations(5));
            intake.rollerVoltage(8);
        }
    }

    public static class IntakeDown extends CommandBase{
        Intake intake;
        public IntakeDown(Intake intake){
            this.intake = intake;
            addRequirements(intake);
        }

        @Override
        public void initialize() {
            intake.setPivotAngle(Rotation2d.fromRotations(24));
            intake.rollerVoltage(8);
        }
    }





}
