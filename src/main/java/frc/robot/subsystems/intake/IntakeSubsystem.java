package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

  private IntakeIO intakeIO;
  private IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();

  public IntakeSubsystem(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;

    setDefaultCommand(
        Commands.parallel(
            setTargetPosition(IntakeConstants.intakeUpPosition), setWheelsVoltage(0)));
  }

  public Command setTargetPosition(double position) {
    return Commands.runOnce(
        () -> {
          intakeIO.setPivotPosition(position);
        });
  }

  public Command setWheelsVoltage(double voltage) {
    return Commands.run(
        () -> {
          intakeIO.setWheelsVoltage(voltage);
        });
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(intakeInputs);

    Logger.processInputs("RealOutputs/Elevator", intakeInputs);
  }
}
