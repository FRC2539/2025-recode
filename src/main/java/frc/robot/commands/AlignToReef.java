package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class AlignToReef extends Command {

  private CommandSwerveDrivetrain drivetrain;

  private DoubleSupplier xVelocity;
  private DoubleSupplier yVelocity;

  private PIDController rotationController = new PIDController(0, 0, 0);

  private PIDController yController = new PIDController(0, 0, 0);

  private Pose2d targetPose;
  private double offset;
  private Rotation2d rotationOffset;

  public AlignToReef(
      DoubleSupplier xVelocity,
      DoubleSupplier yVelocity,
      double alignmentOffset,
      Pose2d alignmentPose,
      Rotation2d rotationOffset) {
    this.xVelocity = xVelocity;
    this.yVelocity = yVelocity;
    this.offset = alignmentOffset;
    this.targetPose = alignmentPose;
    this.rotationOffset = rotationOffset;
  }

  public void initialize() {
    Pose2d CurrentPose = drivetrain.getState().Pose;
    Logger.recordOutput("/AutoAlign/CurrentPose", CurrentPose);
    Logger.recordOutput("/AutoAlign/TargetPose", targetPose);
    Logger.recordOutput("/AutoAlign/Offset", offset);

    rotationController.setSetpoint(rotationOffset.getRadians());
    yController.setSetpoint(offset);
    rotationController.enableContinuousInput(0, 0);
    rotationController.setTolerance(Units.degreesToRadians(0));
    yController.setTolerance(Units.inchesToMeters(0));
  }
}
