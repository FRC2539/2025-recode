package frc.robot.subsystems.gripper;

import org.littletonrobotics.junction.AutoLog;

public interface GripperIO {

  public void updateInputs(GripperIOInputs inputs);

  @AutoLog
  public class GripperIOInputs {

    public double voltage = 0.0;
    public double speed = 0.0;

    public boolean hasPiece = false;
  }

  public void setVoltage(double voltage);

  public boolean hasPiece();
}
