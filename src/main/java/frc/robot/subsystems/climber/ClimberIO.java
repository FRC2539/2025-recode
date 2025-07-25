package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  public void updateInputs(ClimberIOInputs inputs);

  @AutoLog
  public class ClimberIOInputs {

    public double voltage = 0;
    public double position = 0;
    public double speed = 0;
    public double current = 0;
  }

  public void setPosition(double position);

  public void setVoltage(double voltage);
}
