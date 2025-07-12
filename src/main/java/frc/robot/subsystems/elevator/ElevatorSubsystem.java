package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

  private ElevatorIO elevatorIO;

  public double target = 0;

  public ElevatorSubsystem(ElevatorIO elevatorIO) {
    this.elevatorIO = elevatorIO;
  }

  public void setVoltage(double voltage) {
    elevatorIO.setVoltage(voltage);
  }

  public void setPosition(double position) {
    elevatorIO.setPosition(position);
  }

  // public void resetPosition() {
  //   elevatorIO.setPosition(0);
  // }
}
