package frc.robot.constants;

public class GripperConstants {
  public static final int gripperMotorID = 0;
  public static final int gripperSensorAddress = 0;

  public static final double gripperPlacementVoltage = -3;
  public static final double targetSensorConfidence = 0.0; // tune

  public static final double proximityThreshold = 1.0;
  public static final double intakeVoltage = 12;

  public static final boolean Coral = true;
  public static final boolean Algae = false;

  public enum Piece {
    CORAL,
    ALGAE,
    NONE
  }
}
