package frc.robot.subsystems.roller;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {

  public void updateInputs(RollerIOInputs inputs);

  @AutoLog
  public class RollerIOInputs {
    
    double wheelsVoltage = 0;
  }

  public void setWheelsVoltage(double voltage);

}
