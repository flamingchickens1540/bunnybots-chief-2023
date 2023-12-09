package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.Limelight;

public class ShooterCommands {
  public static class Shoot extends Command {

    Shooter shooter;
    double mainRPM;
    double secondaryRPM;

    Limelight limelight;

    Indexer indexer;

    enum State {
      INIT,
      SPINUP,
      SHOOTING,
      DONE
    }

    State state = State.SPINUP;

    public Shoot(Shooter shooter, Limelight limelight, Indexer indexer) {
      this.shooter = shooter;
      addRequirements(shooter, indexer);

      this.limelight = limelight;
      this.indexer = indexer;
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
            indexer.indexerRun(true);
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
public void end(boolean isInterrupted) {
  indexer.indexerRun(false);
}
    @Override
    public boolean isFinished() {
      return state == State.DONE;
    }
  }

  public static class ShooterIdle extends Command {
    Shooter shooter;

    private final double mainIdleRPM = 8000;
    private final double secondaryIdleRPM = 4000;
    private final double loadingIdleVoltage = -8;

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
}
