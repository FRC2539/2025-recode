package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.LimelightHelpers;

public class AutoIntakeCoral extends Command {
    private CommandSwerveDrivetrain drivetrain; 
    private String cameraName;

    private ProfiledPIDController thetaController =
      new ProfiledPIDController(
          2.5,
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
    }

    @Override
    public void initialize() {
        thetaController.setGoal(20);
        thetaController.setTolerance(Math.toRadians(4));
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = new ChassisSpeeds();

        double currentTx = LimelightHelpers.getTX(cameraName);

        if (LimelightHelpers.getTV(cameraName)) {
            speeds.omegaRadiansPerSecond = thetaController.calculate(currentTx);
        }

        if (thetaController.atGoal()) {
            speeds.vxMetersPerSecond = 0.5;
        }

        drivetrain.setControl(m_applyRobotSpeeds.withSpeeds(speeds));
    }
}
