package frc.robot.subsystems.indexer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.util.Units;

public class IndexerIOReal implements IndexerIO {
    CANSparkMax indexerMotor = new CANSparkMax(15, MotorType.kBrushless);
    private static final double GEAR_RATIO = 12.0;


    public IndexerIOReal(){
        indexerMotor.setSmartCurrentLimit(20);
        indexerMotor.setInverted(true);
    }
    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.currentAmps = indexerMotor.getOutputCurrent();
        inputs.appliedVolts = indexerMotor.getAppliedOutput();
        inputs.velocityRadPerSec = Units.rotationsToRadians(indexerMotor.getEncoder().getVelocity() / GEAR_RATIO);
    }

    @Override
    public void setVoltage(double volts) {
        indexerMotor.setVoltage(volts);
    }

    @Override
    public void stop() {
        indexerMotor.setVoltage(0);
    }
}