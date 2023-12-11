package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;

import java.util.logging.Logger;

public class GyroIONavx implements GyroIO{
    private final AHRS navx = new AHRS(SPI.Port.kMXP);
    private Rotation2d lastAngle;
    private double time;

    public GyroIONavx(){
        lastAngle = navx.getRotation2d();
        time = Timer.getFPGATimestamp();
    }


    @Override
    public void updateInputs(GyroIOInputs inputs) {
        double lastTime = inputs.time;
        Rotation2d angle = navx.getRotation2d();
        inputs.time = Timer.getFPGATimestamp();
        inputs.connected = true;
        inputs.yawPosition = -navx.getYaw();
        inputs.yawVelocityRadPerSec = (angle.minus(lastAngle).getRadians())/(inputs.time - lastTime);
        lastAngle = angle;
    }
}
