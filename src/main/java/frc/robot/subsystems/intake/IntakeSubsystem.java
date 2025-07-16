package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private IntakeIO intakeIO;

  public IntakeSubsystem(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;

    setDefaultCommand(setWheelsVoltage(0));
  }

  public Command setTargetPosition(double position) {
    return Commands.runOnce(
        () -> {
          intakeIO.setPivotPosition(position);
        });
  }

  public Command setWheelsVoltage(double voltage) {
    return Commands.runOnce(
        () -> {
          intakeIO.setWheelsVoltage(voltage);
        });
  }
}
