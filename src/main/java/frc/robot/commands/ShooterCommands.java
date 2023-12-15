package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexTemp;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class ShooterCommands {
  public static class Shoot extends CommandBase {

    Shooter shooter;
    double mainRPM;
    double secondaryRPM;

    Limelight limelight;

    enum State {
      INIT,
      SPINUP,
      SHOOTING,
      DONE
    }

    State state = State.SPINUP;

    public Shoot(Shooter shooter, Limelight limelight) {
      this.shooter = shooter;
      addRequirements(shooter);

      this.limelight = limelight;
    }

    @Override
    public void execute() {
      switch (state) {
        case INIT -> {
          shooter.setLoadingVoltage(-8);
          state = State.SPINUP;
        }
        case SPINUP -> {
          //TODO calculation based on distance;

          shooter.setMainVelocity(mainRPM);
          shooter.setSecondaryVelocity(secondaryRPM);
          if (shooter.getMainVelocityRPM() == mainRPM
              && shooter.getSecondaryVelocityRPM() == secondaryRPM
              && limelight.aimed()) {
            shooter.setLoadingVoltage(10);
            state = State.SHOOTING;
          }
        }
        case SHOOTING -> {
          if (true) {
            state = State.DONE;
          }
        }
      }
    }

    @Override
    public boolean isFinished() {
      return state == State.DONE;
    }
  }

  public static class ShooterIdle extends CommandBase {
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

  public static class ShooterTesting extends CommandBase{
    Shooter shooter;
    LoggedDashboardNumber mainRPM = new LoggedDashboardNumber("mainRPM", 0);
    LoggedDashboardNumber secondaryRPM = new LoggedDashboardNumber("secondaryRPM", 0);
    
    LoggedDashboardNumber loadingVoltage = new LoggedDashboardNumber("loadingVoltage",0);
    LoggedDashboardNumber indexPercent = new LoggedDashboardNumber("indexPercent",0);
    IndexTemp index = new IndexTemp();
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
      index.setPercent(indexPercent.get());
    }

    @Override
    public void end(boolean interrupted) {
      shooter.stopAll();
      index.setPercent(0);
    }
  }
}
