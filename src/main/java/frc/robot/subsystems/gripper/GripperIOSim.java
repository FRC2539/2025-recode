package frc.robot.subsystems.gripper;

public class GripperIOSim implements GripperIO {

  private double voltage;
  private double speed;
  private boolean hasPiece = false;

  public void updateInputs(GripperIOInputs inputs) {
    inputs.voltage = voltage;
    inputs.speed = speed;
    inputs.hasPiece = hasPiece;
  }

  public void setVoltage(double voltage) {
    this.voltage = voltage;
  }

  public void setSpeed(double speed) {
    this.speed = speed;
  }

  public void setHasPiece(boolean hasPiece) {
    this.hasPiece = hasPiece;
  }

  public boolean hasPiece() {
    return hasPiece;
  }
  ;
}
