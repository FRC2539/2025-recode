package frc.robot.subsystems.roller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class RollerSubsystem extends SubsystemBase {

  private RollerIO rollerIO;
  private RollerIOInputsAutoLogged rollerInputs = new RollerIOInputsAutoLogged();

  public RollerSubsystem(RollerIO rollerIO) {
    this.rollerIO = rollerIO;

    setDefaultCommand(Commands.parallel(setWheelsVoltage(0)));
  }

  public Command setWheelsVoltage(double voltage) {
    return Commands.run(
        () -> {
          rollerIO.setWheelsVoltage(voltage);
        },
        this);
  }

  @Override
  public void periodic() {
    rollerIO.updateInputs(rollerInputs);

    Logger.processInputs("RealOutputs/Roller", rollerInputs);
  }
}
