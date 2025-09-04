package frc.robot.constants;

import edu.wpi.first.wpilibj.util.Color;

public class GripperConstants {
  public static final int gripperMotorID = 0;
  public static final int gripperSensorAddress = 0;

  public static final double gripperPlacementVoltage = -3;
  public static final double targetSensorConfidence = 0.0; // tune

  public static final double proximityThreshold = 1.0;
  public static final double intakeVoltage = 12;

  public static final Color coralColor = new Color(0.198, 0.443, 0.358);
  public static final Color algaeColor = new Color(0.149, .541, .309);

  public enum Piece {
    CORAL,
    ALGAE,
    NONE
  }
}
