package frc.robot.subsystems.indexer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IndexerIOReal implements IndexerIO {
    CANSparkMax indexerMotor = new CANSparkMax(15, MotorType.kBrushless);

    @Override
    public void setPercent(double percentage) {
        indexerMotor.set(percentage);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.velocityRotationsPerSecond = indexerMotor.getEncoder().getVelocity();
    }
}
