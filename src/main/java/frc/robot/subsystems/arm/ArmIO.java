package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

  public void updateInputs(ArmIOInputs inputs);

  @AutoLog
  public class ArmIOInputs {
    double voltage = 0.0;
    double position = 0.0;
    double temperature = 0.0;
  }

  public void setPosition(double position);

  public void setVoltage(double voltage);
}
