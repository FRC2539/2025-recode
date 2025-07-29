package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    double targetX = 0;
    double targetY = 0;
    boolean hasTarget = false;

    LimelightHelpers.RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");
  }

  public void updateInputs(VisionIOInputs inputs);

  public Pose3d getPoseEstimate();
}
