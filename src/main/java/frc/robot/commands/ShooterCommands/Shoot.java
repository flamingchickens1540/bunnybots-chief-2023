package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.shooter.Shooter;

public class Shoot extends CommandBase {

    Shooter shooter;
    double mainRPM;
    double secondaryRPM;

    double lockMainRPM;
    int counter = 0;

    Limelight limelight;

    enum State {
        INIT,
        SPINUP,
        SHOOTING,
        WINDOWN,
        DONE
    }

    State state = State.SPINUP;

    public Shoot(Shooter shooter, Limelight limelight) {
        this.shooter = shooter;
        addRequirements(shooter);

        this.limelight = limelight;
        System.out.println("I DID A TING");
    }

    @Override
    public void initialize() {
        state = State.INIT;
        counter = 0;
    }

    @Override
    public void execute() {
        switch (state) {
            case INIT -> {
//          indexTemp.setPercent(.75);
                shooter.setLoadingVoltage(0);
                state = State.SPINUP;
            }
            case SPINUP -> {
                //TODO calculation based on distance;
                mainRPM = 3500;
                secondaryRPM = 2000;

                shooter.setMainVelocity(mainRPM);
                shooter.setSecondaryVelocity(secondaryRPM);
                if (shooter.atSetpoints()
//                        ){
//            indexTemp.setPercent(-.75);
              && limelight.aimed()) {
                    shooter.setLoadingVoltage(4);
                    state = State.SHOOTING;
                    lockMainRPM = mainRPM;
                }
            }
            case SHOOTING -> {
                counter += 1;
                if (counter == 12){
                    shooter.setLoadingVoltage(0);
                    state = State.WINDOWN;
                }
            }
            case WINDOWN -> {
                counter += 1;
                if(counter == 16){
                    shooter.setLoadingVoltage(-6);
                }
                if(counter == 21){
                    shooter.setLoadingVoltage(0);
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

