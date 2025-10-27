package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {

  private ClimberIO climberIO;
  private ClimberIOInputsAutoLogged climberInputs = new ClimberIOInputsAutoLogged();

  public ClimberSubsystem(ClimberIO climberIO) {
    this.climberIO = climberIO;

    setDefaultCommand(stopClimber());
  }

  @Override
  public void periodic() {
    climberIO.updateInputs(climberInputs);
    Logger.processInputs("RealOutputs/Climber", climberInputs);
  }

  public Command stopClimber() {
    return Commands.run(
        () -> {
          climberIO.setVoltage(0.0);
        },
        this);
  }

  public Command zeroClimber() {
    return Commands.runOnce(
        () -> {
          climberIO.setPosition(0);
        },
        this);
  }

  public Command moveUpVoltage(double voltage) {
    return setVoltage(voltage);
  }

  public Command moveDownVoltage(double voltage) {
    return setVoltage(-voltage);
  }

  public Command setUpperPosition() {
    return setPosition(ClimberConstants.upperLimit);
  }

  public Command setLowerPosition() {
    return setPosition(ClimberConstants.lowerLimit);
  }

  public Command setVoltage(double voltage) {
    return Commands.run(
        () -> {
          climberIO.setVoltage(voltage);
        },
        this);
  }

  public Command setPosition(double position) {
    return Commands.run(
        () -> {
          climberIO.setPosition(position);
        },
        this);
  }

  public double getPosition() {
    return climberInputs.position;
  }
}
