package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.AverageFilter;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final boolean TUNING = true;
  private final FlywheelIO mainWheelIO;
  private final FlywheelIOInputsAutoLogged mainWheelInputs = new FlywheelIOInputsAutoLogged();
  private final FlywheelIO secondaryWheelIO;
  private final FlywheelIOInputsAutoLogged secondaryWheelInputs = new FlywheelIOInputsAutoLogged();
  private final FlywheelIO loadingWheelIO;
  private final FlywheelIOInputsAutoLogged loadingWheelInputs = new FlywheelIOInputsAutoLogged();

  private final LoggedTunableNumber kPMain = new LoggedTunableNumber("Shooter/kPMain", 0);
  private final LoggedTunableNumber kIMain = new LoggedTunableNumber("Shooter/kIMain", 0);
  private final LoggedTunableNumber kDMain = new LoggedTunableNumber("Shooter/kDMain", 0.001);

  private final SimpleMotorFeedforward mainff = new SimpleMotorFeedforward(-0.02528, 0.01320);

  private final LoggedTunableNumber kPSecondary = new LoggedTunableNumber("Shooter/kPSecondary", 0);
  private final LoggedTunableNumber kISecondary = new LoggedTunableNumber("Shooter/kISecondary", 0);
  private final LoggedTunableNumber kDSecondary = new LoggedTunableNumber("Shooter/kDSecondary", 0);
  private final SimpleMotorFeedforward secondaryff = new SimpleMotorFeedforward(0.01093, 0.01857);

  private double mainSetpoint = 0;
  private double secondarySetpoint = 0;

  private AverageFilter mainFilter = new AverageFilter(5);
  private AverageFilter secondaryFilter = new AverageFilter(5);
  public Shooter(FlywheelIO mainWheelIO, FlywheelIO secondaryWheelIO, FlywheelIO loadingWheelIO) {
    this.mainWheelIO = mainWheelIO;
    this.secondaryWheelIO = secondaryWheelIO;
    this.loadingWheelIO = loadingWheelIO;
  }

  @Override
  public void periodic() {
    mainWheelIO.updateInputs(mainWheelInputs);
    secondaryWheelIO.updateInputs(secondaryWheelInputs);
    loadingWheelIO.updateInputs(loadingWheelInputs);

    mainFilter.add(getMainVelocityRPM());
    secondaryFilter.add(getSecondaryVelocityRPM());

    Logger.getInstance().processInputs("Shooter/main", mainWheelInputs);
    Logger.getInstance().processInputs("Shooter/secondary", secondaryWheelInputs);
    Logger.getInstance().processInputs("Shooter/loading", loadingWheelInputs);


    Logger.getInstance().recordOutput("Shooter/mainSetpoint", mainSetpoint);
    Logger.getInstance().recordOutput("Shooter/secondarySetpoint", secondarySetpoint);
    Logger.getInstance().recordOutput("Shooter/mainVelocityRPM", getMainVelocityRPM());
    Logger.getInstance().recordOutput("Shooter/secondaryVelocityRPM", getSecondaryVelocityRPM());
    Logger.getInstance().recordOutput("Shooter/mainAverageVelocityRPM", mainFilter.getAverage());
    Logger.getInstance().recordOutput("Shooter/secondaryAverageVelocityRPM", secondaryFilter.getAverage());

    if (TUNING) {
      if (kPMain.hasChanged() || kIMain.hasChanged() || kDMain.hasChanged()) {
        mainWheelIO.configurePID(kPMain.get(), kIMain.get(), kDMain.get());
      }
      if (kPSecondary.hasChanged() || kISecondary.hasChanged() || kDSecondary.hasChanged()) {
        mainWheelIO.configurePID(kPSecondary.get(), kISecondary.get(), kDSecondary.get());
      }
    }


  }

  public void setMainVelocity(double RPM) {
    double rpmRadSec = Units.rotationsPerMinuteToRadiansPerSecond(RPM);
    mainSetpoint = RPM;
    mainWheelIO.setVelocity(rpmRadSec, mainff.calculate(rpmRadSec));
  }

  public void setSecondaryVelocity(double RPM) {
    double rpmRadSec = Units.rotationsPerMinuteToRadiansPerSecond(RPM);
    secondarySetpoint = RPM;
    secondaryWheelIO.setVelocity(rpmRadSec, secondaryff.calculate(rpmRadSec));
  }

  public void setMainVoltage(double volts) {
    mainWheelIO.setVoltage(volts);
  }

  public void setSecondaryVoltage(double volts) {
    secondaryWheelIO.setVoltage(volts);
  }

  public void setLoadingVoltage(double volts) {
    loadingWheelIO.setVoltage(volts);
  }

  public void stopAll() {
    mainWheelIO.stop();
    secondaryWheelIO.stop();
    loadingWheelIO.stop();
  }

  /** Runs forwards at the commanded voltage. */
  public void runMainCharacterizationVolts(double volts) {
    mainWheelIO.setVoltage(volts);
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getMainCharacterizationVelocity() {
    return mainWheelInputs.velocityRadPerSec;
  }

  /** Runs forwards at the commanded voltage. */
  public void runSecondaryCharacterizationVolts(double volts) {
    secondaryWheelIO.setVoltage(volts);
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getSecondaryCharacterizationVelocity() {
    return secondaryWheelInputs.velocityRadPerSec;
  }


  public double getMainVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(mainWheelInputs.velocityRadPerSec);
  }

  public double getSecondaryVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(secondaryWheelInputs.velocityRadPerSec);
  }

  public boolean mainAtSetpoint(){
    return mainFilter.atSetpoint(mainSetpoint, 50);
  }
  public boolean secondaryAtSetpoint(){
    return secondaryFilter.atSetpoint(secondarySetpoint, 50);
  }
  public boolean atSetpoints(){
    return mainAtSetpoint() && secondaryAtSetpoint();
  }

}
