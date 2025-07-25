package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  public void updateInputs(ElevatorIOInputs inputs);

  @AutoLog
  public class ElevatorIOInputs {
    public double position = 0;
    public double speed = 0;
    public double voltage;
    public double temperature = 0;
    public double current = 0;
  }

  public void setPosition(double position);

  public void setVoltage(double voltage);

  public boolean isAtSetpoint();
}
