package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  public void updateInputs(IntakeIOInputs inputs);

  @AutoLog
  public class IntakeIOInputs {
    double pivotVoltage = 0;
    double pivotPosition = 0;
    double wheelsVoltage = 0;
  }

  public void setWheelsVoltage(double voltage);

  public void setPivotPosition(double position);
}
