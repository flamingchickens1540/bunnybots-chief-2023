package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterIdle extends CommandBase {
    Shooter shooter;

    private final double mainIdleRPM = 4000;
    private final double secondaryIdleRPM = 0;
    private final double loadingIdleVoltage = 0;

    public ShooterIdle(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setMainVelocity(mainIdleRPM);
        shooter.setSecondaryVelocity(secondaryIdleRPM);
        shooter.setLoadingVoltage(loadingIdleVoltage);
    }
}

