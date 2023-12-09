package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexer.Indexer;

public class IndexerDefaultCommand extends CommandBase {
    private final Indexer indexer;


    public IndexerDefaultCommand (Indexer indexer) {
        this.indexer = indexer;
        addRequirements(indexer);
    }


    @Override
    public void initialize() {}

    @Override
    public void execute() {
        indexer.indexerRun(false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
