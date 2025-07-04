package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {

  private ElevatorIO pivotIO;
  private ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();

  private SysIdRoutine elevatorSysIdRoutine = // tune
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(4),
              null,
              state -> Logger.recordOutput("Elevator/SysIdElevator_State", state.toString())),
          new SysIdRoutine.Mechanism(
              (voltage) -> pivotIO.setVoltage(voltage.in(Volts)), null, this));

  public ElevatorSubsystem(ElevatorIO elevatorIO) {
    this.pivotIO = elevatorIO;
    setDefaultCommand(setVoltage(0));
  }

  public void periodic() {

    pivotIO.updateInputs(elevatorInputs);

    Logger.processInputs("RealOutputs/Elevator", elevatorInputs);
  }

  public Command runQStaticElevatorSysId(SysIdRoutine.Direction direction) {
    return elevatorSysIdRoutine.quasistatic(direction);
  }

  public Command runDynamicElevatorSysId(SysIdRoutine.Direction direction) {
    return elevatorSysIdRoutine.dynamic(direction);
  }

  public Command moveElevatorUp() { // tune
    return setVoltage(12);
  }

  public Command moveElevatorDown() { // tune
    return setVoltage(-12);
  }

  public Command setVoltage(double voltage) {
    return run(
        () -> {
          pivotIO.setVoltage(voltage);
        });
  }

  public Command setPosition(double position) {
    return Commands.runOnce(
        () -> {
          pivotIO.setPosition(position);
        },
        this);
  }

  public double getPosition() {
    return elevatorInputs.position;
  }
}
