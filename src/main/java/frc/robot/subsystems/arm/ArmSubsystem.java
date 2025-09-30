package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ArmSubsystem extends SubsystemBase {

  public ArmIO armIO;
  private ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();
  public LoggedNetworkNumber armTuneables = new LoggedNetworkNumber("arm tuneable", 0);
  private double positionSetpoint = 0.0;

  public ArmSubsystem(ArmIO armIO) {
    this.armIO = armIO;
  }

  private SysIdRoutine armSysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(4),
              null,
              state -> Logger.recordOutput("Arm/SysIdArm_State", state.toString())),
          new SysIdRoutine.Mechanism((voltage) -> armIO.setVoltage(voltage.in(Volts)), null, this));

  public Command runQStaticArmSysId(SysIdRoutine.Direction direction) {
    return armSysIdRoutine.quasistatic(direction);
  }

  @Override
  public void periodic() {
    armIO.updateInputs(armInputs);
    Logger.processInputs("RealOutputs/Arm", armInputs);
  }

  public Command runDynamicArmSysId(SysIdRoutine.Direction direction) {
    return armSysIdRoutine.dynamic(direction);
  }

  public Command tuneableVoltage() {
    return run(() -> armIO.setVoltage(armTuneables.get()));
  }

  public void setPosition(double position) {
    // System.out.println("Setting arm position to: " + position);
    this.positionSetpoint = position;
    armIO.setPosition(position);
  }

  public void setVoltage(double voltage) {
    armIO.setVoltage(voltage);
  }

  public boolean isAtSetpoint() {
    return armIO.isAtSetpoint();
  }

  public Command goToPositionCommand(double position) {
    return Commands.runOnce(() -> setPosition(position), this)
        .andThen(
            Commands.run(() -> {}, this) // Runs an empty action but requires the subsystem
                .until(this::isAtSetpoint) // Checks the setpoint using a method reference for
            // robustness
            );
  }

  public double getPositionSetpoint() {
    return positionSetpoint;
  }
}
