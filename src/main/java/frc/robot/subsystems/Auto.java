package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class Auto extends SubsystemBase {
    CommandSwerveDrivetrain drivetrain;
  private RobotConfig config;


  private final SwerveRequest.ApplyRobotSpeeds m_applyRobotSpeeds =
            new SwerveRequest.ApplyRobotSpeeds()
                    .withDriveRequestType(DriveRequestType.Velocity)
                    .withSteerRequestType(SteerRequestType.Position);
                    
  public Auto() {
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }

    configureAutoBuilder();


  }

  public void configureAutoBuilder() {
    AutoBuilder.configure(
            drivetrain::getRobotPose, // Robot pose supplier
            drivetrain::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            drivetrain::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> drivetrain.setControl(m_applyRobotSpeeds.withSpeeds(speeds)
                .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }
}
