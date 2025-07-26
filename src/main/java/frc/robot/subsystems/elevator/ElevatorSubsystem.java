package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {

  private ElevatorIO elevatorIO;
  private ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();

  public ElevatorSubsystem(ElevatorIO elevatorIO) {
    this.elevatorIO = elevatorIO;
  }

  private SysIdRoutine elevatorSysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(4),
              null,
              state -> Logger.recordOutput("Elevator/SysIdElevator_State", state.toString())),
          new SysIdRoutine.Mechanism(
              (voltage) -> elevatorIO.setVoltage(voltage.in(Volts)), null, this));

  public Command runQStaticElevatorSysId(SysIdRoutine.Direction direction) {
    return elevatorSysIdRoutine.quasistatic(direction);
  }

  public Command runDynamicElevatorSysId(SysIdRoutine.Direction direction) {
    return elevatorSysIdRoutine.dynamic(direction);
  }

  public void setVoltage(double voltage) {
    elevatorIO.setVoltage(voltage);
  }

  public void setPosition(double position) {
    elevatorIO.setPosition(position);
  }

  public double getPosition() {
    return elevatorInputs.position;
  }

  // public void resetPosition() {
  //   elevatorIO.setPosition(0);
  // }

  @Override
  public void periodic() {

    elevatorIO.updateInputs(elevatorInputs);

    Logger.processInputs("RealOutputs/Elevator", elevatorInputs);
  }

  public boolean isAtSetpoint() {
    return elevatorIO.isAtSetpoint();
  }
}
