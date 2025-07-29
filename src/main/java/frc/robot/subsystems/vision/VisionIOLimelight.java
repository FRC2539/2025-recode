package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.Supplier;

public class VisionIOLimelight implements VisionIO {

  String cameraName = "";
  Supplier<Rotation2d> currentHeading;

  public VisionIOLimelight(String cameraName, Supplier<Rotation2d> currentHeading) {
    this.cameraName = cameraName;
    this.currentHeading = currentHeading;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.targetX = LimelightHelpers.getTX(cameraName);
    inputs.targetY = LimelightHelpers.getTY(cameraName);
    inputs.hasTarget = LimelightHelpers.getTV(cameraName);

    LimelightHelpers.Flush();
  }

  @Override
  public Pose3d getPoseEstimate() {
    return LimelightHelpers.getBotPose3d_wpiBlue(cameraName);
  }
}
