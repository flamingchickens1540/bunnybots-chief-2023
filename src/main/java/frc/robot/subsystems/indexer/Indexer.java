package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    IndexerIO io;
    IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    public Indexer(IndexerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }


    public void setVoltage(double volts){io.setVoltage(volts);}
}