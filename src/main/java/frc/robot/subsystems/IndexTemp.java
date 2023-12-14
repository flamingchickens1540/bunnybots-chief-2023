package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IndexTemp extends SubsystemBase {
    CANSparkMax spark = new CANSparkMax(15, CANSparkMaxLowLevel.MotorType.kBrushless);

    public void setPercent(double percent){
        spark.set(percent);
    }

    @Override
    public void periodic() {
        Logger.getInstance().recordOutput("Indexer/outputCurrent", spark.getOutputCurrent());
    }
}
