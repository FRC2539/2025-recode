package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.LimelightHelpers;

public class AutoIntakeCoral extends Command {
  private CommandSwerveDrivetrain drivetrain;
  private String cameraName;
  private boolean hasRotated = false;
  private ProfiledPIDController thetaController =
      new ProfiledPIDController(
          3,
          0,
          0,
          new TrapezoidProfile.Constraints(
              Math.toRadians(360), // Max velocity (radians per second)
              Math.toRadians(180) // Max acceleration (radians per second squared)
              ));

  private final SwerveRequest.ApplyRobotSpeeds m_applyRobotSpeeds =
      new SwerveRequest.ApplyRobotSpeeds()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  public AutoIntakeCoral(CommandSwerveDrivetrain dt, String cameraName) {
    this.cameraName = cameraName;
    this.drivetrain = dt;
  }

  @Override
  public void initialize() {
    thetaController.setGoal(5.4);
    thetaController.setTolerance(0.5);
  }

  @Override
  public void execute() {
    ChassisSpeeds speeds = new ChassisSpeeds();

    double currentTx = LimelightHelpers.getTX(cameraName);

    System.out.println(thetaController.atSetpoint() + "  " + thetaController.getPositionError());

    // if (LimelightHelpers.getTV(cameraName)) {
    speeds.omegaRadiansPerSecond = Units.degreesToRadians(thetaController.calculate(currentTx));
    // }

    if (thetaController.atSetpoint() || hasRotated) {
      hasRotated = true;
      speeds.vxMetersPerSecond = 0.5;
    }

    drivetrain.setControl(m_applyRobotSpeeds.withSpeeds(speeds));
  }
}
