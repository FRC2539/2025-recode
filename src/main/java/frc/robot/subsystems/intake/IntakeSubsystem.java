package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

  private double positionSetpoint = 0.0;

  private IntakeIO intakeIO;
  private IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();

  public IntakeSubsystem(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;

    setDefaultCommand(
        Commands.parallel(
            setTargetPosition(IntakeConstants.intakeUpPosition))); // setWheelsVoltage(0)
  }

  public Command setTargetPosition(double position) {
    return Commands.run(
        () -> {
          intakeIO.setPivotPosition(position);
        },
        this);
  }

  // public Command setWheelsVoltage(double voltage) {
  //   return Commands.run(
  //       () -> {
  //         intakeIO.setWheelsVoltage(voltage);
  //       });
  // }

  // public Command setPosition(double position) {
  //   return Commands.sequence(
  //       Commands.runOnce(() -> intakeIO.setPivotPosition(position), this),
  //       Commands.waitUntil(() -> intakeIO.isAtSetpoint()));
  // }

  // Commands.runOnce(() -> intakeIO.setWheelsVoltage(voltage), this));

  public boolean isAtSetpoint() {
    return intakeIO.isAtSetpoint();
  }

  // public Command goToPositionCommand(double position) {
  //   setTargetPosition(position);
  //   return setTargetPosition(position)
  //       .until(this::isAtSetpoint); // Checks the setpoint using a method reference for
  //   // robustness
  //}

    public Command goToPositionCommand(double position) {
      return Commands.runOnce(() -> setTargetPosition(position), this)
          .andThen(Commands.run(() -> {}, this).until(this::isAtSetpoint));
    }

  

  @Override
  public void periodic() {
    intakeIO.updateInputs(intakeInputs);

    Logger.processInputs("RealOutputs/Intake", intakeInputs);
  }
}
