package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.LimelightHelpers;
import org.littletonrobotics.junction.AutoLogOutput;

public class AlignToReefMT2 extends Command {
  private CommandSwerveDrivetrain drive;

  // drive request from on-season code
  private final SwerveRequest.ApplyFieldSpeeds m_applyFieldSpeeds =
      new SwerveRequest.ApplyFieldSpeeds()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.MotionMagicExpo)
          .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

  private ProfiledPIDController thetaController =
      new ProfiledPIDController(
          0.1,
          0,
          0,
          new TrapezoidProfile.Constraints(
              Math.toRadians(360), // Max velocity (radians per second)
              Math.toRadians(180) // Max acceleration (radians per second squared)
              ));
  private PIDController yController = new PIDController(0.1, 0, 0);
  private PIDController xController = new PIDController(0.1, 0, 0);

  @AutoLogOutput private Pose2d targetPose;

  private double xTarget;
  private double yTarget;
  private double rotationTarget;

  public AlignToReefMT2(
      CommandSwerveDrivetrain drivetrain,
      Pose2d tagPose,
      double xOffset,
      double yOffset,
      Rotation2d rotationOffset) {
    this.drive = drivetrain;
    this.xTarget = xOffset;
    this.yTarget = yOffset;
    this.rotationTarget = rotationOffset.getRadians();

    this.targetPose = tagPose;
  }

  @Override
  public void initialize() {

    xController.setSetpoint(xTarget);
    yController.setSetpoint(yTarget);
    thetaController.setGoal(rotationTarget);

    thetaController.setTolerance(Units.degreesToRadians(3));
    yController.setTolerance(Units.inchesToMeters(1));
    xController.setTolerance(Units.inchesToMeters(1));
  }

  @Override
  public void execute() {
    Pose2d currentPose = new Pose2d();

    if (LimelightHelpers.getTA("limelight-left") > LimelightHelpers.getTA("limelight-right")) {
      // botPose = LimelightHelpers.getBotPose_TargetSpace("limelight-left");
      currentPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left").pose;
    } else {
      // botPose = LimelightHelpers.getBotPose_TargetSpace("limelight-right");
      currentPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-right").pose;
    }

    Transform2d offset = currentPose.minus(targetPose);

    double xVelocity = xController.calculate(offset.getX());
    double yVelocity = yController.calculate(offset.getY());
    double thetaVelocity = thetaController.calculate(offset.getRotation().getRadians());

    // System.out.println("x error: " + xController.getError() + " y error:" +
    // yController.getError() + " rotation error: " + thetaController.getPositionError());

    // PID Controllers are using tag-relative units, convert to field relative for actual driving
    // (code from 2025 on-season auto-align)

    Rotation2d tagRotation = targetPose.getRotation();
    ChassisSpeeds tagRelativeCommandedVelocities =
        new ChassisSpeeds(xVelocity, yVelocity, thetaVelocity);

    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(tagRelativeCommandedVelocities, tagRotation);

    drive.setControl(m_applyFieldSpeeds.withSpeeds(fieldRelativeSpeeds));
  }

  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint();
  }
}
