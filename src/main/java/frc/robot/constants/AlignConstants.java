package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;

public class AlignConstants {
  public static final double Kp = 1;
  public static final double Ki = 0.0;
  public static final double Kd = 0.0;

  public static final double leftOffset = -0.2;
  public static final double rightOffset = 0.2;
  public static final double centerOffset = -0.3;

  public static final double reefDistance23 = Units.inchesToMeters(22);
  public static final double reefDistance4 = Units.inchesToMeters(26);

  public static final double leftAlign = Units.inchesToMeters(-7);
  public static final double rightAlign = Units.inchesToMeters(7);

  public static final double aligningAngleTolerance = Units.degreesToRadians(3);
  public static final double aligningXTolerance = Units.inchesToMeters(2);
  public static final double aligningYTolerance = Units.inchesToMeters(1);

  public static final AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
}
