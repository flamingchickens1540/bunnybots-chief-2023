package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    IndexerIO io;
    IndexerIOInputsAutoLogged inputs;

    public Indexer(IndexerIO io) {
        this.io = io;
    }

    public void indexerRun(boolean shooting) {
        if (shooting == true) {
            io.setPercent(1);
        } else {
            io.setPercent(0.2);
        }
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }
}
