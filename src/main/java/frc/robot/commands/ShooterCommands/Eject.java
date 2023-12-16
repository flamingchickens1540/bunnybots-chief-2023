package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class Eject extends CommandBase {

    Shooter shooter;
    int counter = 0;


    enum State {
        INIT,
        SPINUP,
        SHOOTING,
        WINDOWN,
        DONE
    }

    State state = State.SPINUP;

    public Eject(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
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
                shooter.setMainVelocity(4000);
            }
            case SPINUP -> {
                //TODO calculation based on distance;
                if (shooter.atSetpoints() || shooter.getMainVelocityRPM() > 4000
                        ){
//            indexTemp.setPercent(-.75);
//              && limelight.aimed()) {
                    shooter.setLoadingVoltage(4);
                    state = State.SHOOTING;
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

