package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class ShooterTesting extends CommandBase {
    Shooter shooter;
    LoggedDashboardNumber mainRPM = new LoggedDashboardNumber("mainRPM", 0);
    LoggedDashboardNumber secondaryRPM = new LoggedDashboardNumber("secondaryRPM", 0);

    LoggedDashboardNumber loadingVoltage = new LoggedDashboardNumber("loadingVoltage",0);
    LoggedDashboardNumber indexPercent = new LoggedDashboardNumber("indexPercent",0);
    public ShooterTesting(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
//      shooter.setMainVelocity(mainRPM.get());
        shooter.setMainVelocity(mainRPM.get());
        shooter.setSecondaryVelocity(secondaryRPM.get());
        shooter.setLoadingVoltage(loadingVoltage.get());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopAll();
    }
}

