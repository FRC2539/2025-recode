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
        });
  }

  public Command runBothWheelsBackwards(double voltage) {
    return Commands.runOnce(
        () -> {
          straightenatorIO.setLeftMotorVoltage(-3);
          straightenatorIO.setRightMotorVoltage(-3);
        });
  }

  public Command unJam(double voltage) {
    return Commands.runOnce(
        () -> {
          straightenatorIO.setLeftMotorVoltage(StraightenatorConstants.ejectLeftVoltage);
          straightenatorIO.setRightMotorVoltage(StraightenatorConstants.ejectRightVoltage);
        });
  }

  @Override
  public void periodic() {

    straightenatorIO.updateInputs(straightenatorInputs);

    Logger.processInputs("RealOutputs/Elevator", straightenatorInputs);
  }

  public boolean isCradled() {
    return straightenatorInputs.isCradled;
  }
}
