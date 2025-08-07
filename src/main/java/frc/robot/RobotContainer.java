// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.TunerConstants;
import frc.robot.lib.controller.LogitechController;
import frc.robot.lib.controller.ThrustmasterJoystick;
import frc.robot.subsystems.arm.ArmIOTalonFX;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.gripper.GripperIOTalonFX;
import frc.robot.subsystems.gripper.GripperSubsystem;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.straightenator.StraightenatorSubsystem;
import frc.robot.subsystems.straightenator.StraightenatorTalonFX;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.Position;
import frc.robot.subsystems.superstructure.Superstructure.ScoringMode;
import frc.robot.subsystems.vision.VisionIO;
import java.util.Set;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond);

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.Velocity);

  // Controller
  private final ThrustmasterJoystick leftJoystick = new ThrustmasterJoystick(0);
  private final ThrustmasterJoystick rightJoystick = new ThrustmasterJoystick(1);
  private final LogitechController operatorController = new LogitechController(2);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  public final ElevatorSubsystem elevator;
  public final ArmSubsystem arm;
  public final IntakeSubsystem intake;
  public final StraightenatorSubsystem straightenator;
  public final ClimberSubsystem climber;
  public final Superstructure superstructure;
  public final GripperSubsystem gripper;
  public VisionIO vision;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    if (Robot.isReal()) {

      elevator = new ElevatorSubsystem(new ElevatorIOTalonFX());
      arm = new ArmSubsystem(new ArmIOTalonFX());
      climber = new ClimberSubsystem(new ClimberIOTalonFX());
      intake = new IntakeSubsystem(new IntakeIOTalonFX());
      straightenator = new StraightenatorSubsystem(new StraightenatorTalonFX());
      gripper = new GripperSubsystem(new GripperIOTalonFX());

      // camera = new VisionSubsystem((Pose2d visionRobotPoseMeters, double timestampSeconds,
      // Matrix<N3, N1> visionMeasurementStdDevs) -> {
      //         drivetrain.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds,
      // visionMeasurementStdDevs);
      //     }, new VisionIOLimelight("limelight", () -> drivetrain.getPigeon2().getRotation2d()));
    } else {
      elevator = new ElevatorSubsystem(null);
      arm = new ArmSubsystem(null);
      climber = new ClimberSubsystem(null);
      intake = new IntakeSubsystem(null);
      straightenator = new StraightenatorSubsystem(null);
      gripper = new GripperSubsystem(null);

      // camera = null;
    }

    superstructure = new Superstructure(elevator, arm, gripper);
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    rightJoystick
        .getLeftTopLeft()
        .onTrue(
            Commands.runOnce(
                () ->
                    drivetrain.resetPose(
                        new Pose2d(0, 0, drivetrain.getOperatorForwardDirection()))));

    rightJoystick
        .getTrigger()
        .onTrue(Commands.defer(() -> superstructure.place(), Set.of(superstructure)));
    leftJoystick
        .getTrigger()
        .onTrue(Commands.defer(() -> superstructure.place(), Set.of(superstructure)));

    rightJoystick.getLeftThumb().whileTrue(superstructure.intakeToCradle());
    leftJoystick.getLeftThumb().whileTrue(superstructure.intakeAlgae(Position.AlgaePickup));

    operatorController
        .getA()
        .and(() -> superstructure.getCurrentScoringMode() == ScoringMode.Coral)
        .onTrue(superstructure.goToLevel(Position.L1));
    operatorController
        .getB()
        .and(() -> superstructure.getCurrentScoringMode() == ScoringMode.Coral)
        .onTrue(superstructure.updateTargetPosition(Position.L2));
    operatorController
        .getX()
        .and(() -> superstructure.getCurrentScoringMode() == ScoringMode.Coral)
        .onTrue(superstructure.updateTargetPosition(Position.L3));
    operatorController
        .getY()
        .and(() -> superstructure.getCurrentScoringMode() == ScoringMode.Coral)
        .onTrue(superstructure.updateTargetPosition(Position.L4));

    operatorController.getStart().onTrue(superstructure.goToLevel(Position.ClimbPosition));

    operatorController.getDPadDown().onTrue(superstructure.goToLevel(Position.CoralHome));

    operatorController
        .getY()
        .and(() -> superstructure.getCurrentScoringMode() == ScoringMode.Algae)
        .onTrue(superstructure.goToLevel(Position.AlgaeNet));
    operatorController
        .getA()
        .and(() -> superstructure.getCurrentScoringMode() == ScoringMode.Algae)
        .onTrue(superstructure.goToLevel(Position.AlgaeProcessor));
    operatorController
        .getB()
        .and(() -> superstructure.getCurrentScoringMode() == ScoringMode.Algae)
        .onTrue(superstructure.intakeAlgae(Position.AlgaeL2));
    operatorController
        .getX()
        .and(() -> superstructure.getCurrentScoringMode() == ScoringMode.Algae)
        .onTrue(superstructure.intakeAlgae(Position.AlgaeL3));

    // Default command, normal field-relative drive

    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityY(
                        -Math.pow(leftJoystick.getXAxis().getRaw(), 3)
                            * MaxSpeed) // Drive forward with negative Y (forward) POSSIBLY READD -
                    // TO FIX ANY INVERT ISSUES
                    .withVelocityX(
                        -Math.pow(leftJoystick.getYAxis().getRaw(), 3)
                            * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        Math.pow(-rightJoystick.getXAxis().getRaw(), 3) * MaxAngularRate)
                    .withDeadband(0.02) // Drive counterclockwise with negative X (left)
            ));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
