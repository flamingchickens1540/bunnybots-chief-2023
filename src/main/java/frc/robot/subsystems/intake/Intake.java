package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;


public class Intake extends SubsystemBase {
  private static final boolean TUNING_MODE = true;
  private final IntakeIO io;

  private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

  private final LoggedTunableNumber kPLead = new LoggedTunableNumber("Intake/kPLead", 0.058);
  private final LoggedTunableNumber kILead = new LoggedTunableNumber("Intake/kILead", 0);
  private final LoggedTunableNumber kDLead = new LoggedTunableNumber("Intake/kDLead", 0.7);

  private final LoggedTunableNumber kPFollow = new LoggedTunableNumber("Intake/kPFollow", 0.045);
  private final LoggedTunableNumber kIFollow = new LoggedTunableNumber("Intake/kIFollow", 0);
  private final LoggedTunableNumber kDFollow = new LoggedTunableNumber("Intake/kDFollow", 4);

//  private final ArmFeedforward ff = new ArmFeedforward()

  private Rotation2d setpoint = new Rotation2d();

  public Intake(IntakeIO io) {
    this.io = io;
    io.setPivotBreakMode(true);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Intake", inputs);

    io.configureLeadPID(0.05, 0, .7);
    io.configureSecondPID(0.045, 0, 4);


    if (TUNING_MODE && (kPLead.hasChanged() || kILead.hasChanged() || kDLead.hasChanged())) {
      io.configureLeadPID(kPLead.get(), kILead.get(), kDLead.get());
    }
    if (TUNING_MODE && (kPFollow.hasChanged() || kIFollow.hasChanged() || kDFollow.hasChanged())) {
    }

    Logger.getInstance().recordOutput("Intake/setpoint", setpoint.getRotations());
  }


  public void rollerVoltage(double volts){
    io.setRollerVoltage(volts);
  }

  public void setPivotAngle(Rotation2d angle){
    System.out.println("Setting Angle Subsystem");
    setpoint = angle;
    io.setPivotPosition(angle);
  }



}
