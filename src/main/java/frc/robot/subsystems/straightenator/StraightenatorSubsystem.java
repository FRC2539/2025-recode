package frc.robot.subsystems.straightenator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.StraightenatorConstants;
import org.littletonrobotics.junction.Logger;

public class StraightenatorSubsystem extends SubsystemBase {

  private StraightenatorIO straightenatorIO;
  private StraightenatorIOInputsAutoLogged straightenatorInputs =
      new StraightenatorIOInputsAutoLogged();

  public StraightenatorSubsystem(StraightenatorIO straightenatorIO) {
    this.straightenatorIO = straightenatorIO;

    setDefaultCommand(setWheelVoltage(0));
  }

  public Command setWheelVoltage(double voltage) {
    return Commands.run(
        () -> {
          straightenatorIO.setVoltage(voltage);
        },
        this);
  }

  public Command runBothWheelsBackwards(double voltage) {
    return Commands.runOnce(
        () -> {
          straightenatorIO.setLeftMotorVoltage(-3);
          straightenatorIO.setRightMotorVoltage(-3);
        },
        this);
  }

  public Command runBothWheelsCorrect(double voltage) {
    return Commands.run(
        () -> {
          straightenatorIO.setLeftMotorVoltage(-4);
          straightenatorIO.setRightMotorVoltage(4);
        },
        this);
  }

  public Command unJam(double voltage) {
    return Commands.runOnce(
        () -> {
          straightenatorIO.setLeftMotorVoltage(StraightenatorConstants.ejectLeftVoltage);
          straightenatorIO.setRightMotorVoltage(StraightenatorConstants.ejectRightVoltage);
        },
        this);
  }

  @Override
  public void periodic() {

    straightenatorIO.updateInputs(straightenatorInputs);

    Logger.processInputs("RealOutputs/Straightenator", straightenatorInputs);
  }

  public boolean isCradled() {
    return straightenatorInputs.isCradled;
  }
}
