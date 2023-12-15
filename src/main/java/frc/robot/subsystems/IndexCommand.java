package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexCommand extends CommandBase {
    IndexTemp index;

    public IndexCommand(IndexTemp index){
       this.index = index;
        addRequirements(index);
    }

    @Override
    public void initialize() {
        index.setPercent(-.75);
    }
}
