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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AlignAndDriveToReef;
import frc.robot.commands.AlignToReef;
import frc.robot.constants.AlignConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.VisionConstants;
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
import frc.robot.subsystems.lights.LightsSubsystem;
import frc.robot.subsystems.lights.LightsSubsystem.LightsControlModule;
import frc.robot.subsystems.roller.RollerIOTalonFX;
import frc.robot.subsystems.roller.RollerSubsystem;
import frc.robot.subsystems.straightenator.StraightenatorSubsystem;
import frc.robot.subsystems.straightenator.StraightenatorTalonFX;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.Position;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.Set;
import java.util.function.DoubleSupplier;

public class RobotContainer {
  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond);

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.05)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.Velocity);

  // Controllers
  private final ThrustmasterJoystick leftJoystick = new ThrustmasterJoystick(0);
  private final ThrustmasterJoystick rightJoystick = new ThrustmasterJoystick(1);
  private final LogitechController operatorController = new LogitechController(2);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  public final ElevatorSubsystem elevator;
  public final ArmSubsystem arm;
  public final RollerSubsystem roller;
  public final IntakeSubsystem intake;
  public final StraightenatorSubsystem straightenator;
  public final ClimberSubsystem climber;
  public final Superstructure superstructure;
  public final GripperSubsystem gripper;
  public final VisionSubsystem vision;
  public final LightsSubsystem lights;
  private DoubleSupplier leftJoystickVelocityX;
  private DoubleSupplier leftJoystickVelocityY;
  private DoubleSupplier rightJoystickVelocityTheta;

  // Dashboard inputs
  // private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    if (Robot.isReal()) {

      elevator = new ElevatorSubsystem(new ElevatorIOTalonFX());
      roller = new RollerSubsystem(new RollerIOTalonFX());
      arm = new ArmSubsystem(new ArmIOTalonFX());
      climber = new ClimberSubsystem(new ClimberIOTalonFX());
      intake = new IntakeSubsystem(new IntakeIOTalonFX());
      straightenator = new StraightenatorSubsystem(new StraightenatorTalonFX());
      gripper = new GripperSubsystem(new GripperIOTalonFX());
      lights = new LightsSubsystem();
      // vision = new VisionSubsystem(drivetrain.addVisionMeasurement(null, MaxAngularRate);,
      // new VisionIOLimelight("", () -> drivetrain.getRotation()));

    } else {
      elevator = new ElevatorSubsystem(null);
      roller = new RollerSubsystem(null);
      arm = new ArmSubsystem(null);
      climber = new ClimberSubsystem(null);
      intake = new IntakeSubsystem(null);
      straightenator = new StraightenatorSubsystem(null);
      gripper = new GripperSubsystem(null);
      lights = new LightsSubsystem();
      // vision = null;
    }
    vision = null;

    superstructure = new Superstructure(elevator, arm, gripper, intake, roller, straightenator);

    configureButtonBindings();
    assembleLightsSuppliers();

    // Set the default command for the arm to hold its current setpoint
    arm.setDefaultCommand(Commands.run(() -> arm.setPosition(arm.getPositionSetpoint()), arm));

    // Set the default command for the elevator to hold its current setpoint
    elevator.setDefaultCommand(
        Commands.run(() -> elevator.setPosition(elevator.getPositionSetpoint()), elevator));

    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityY(
                        -Math.pow(leftJoystick.getXAxis().getRaw(), 3)
                            * MaxSpeed) // Drive forward with negative Y
                    // (forward) POSSIBLY READD -
                    // TO FIX ANY INVERT ISSUES
                    .withVelocityX(
                        -Math.pow(leftJoystick.getYAxis().getRaw(), 3)
                            * MaxSpeed) // Drive left with negative X
                    // (left)
                    .withRotationalRate(
                        Math.pow(-rightJoystick.getXAxis().getRaw(), 3) * MaxAngularRate)
                    .withDeadband(0.02) // Drive counterclockwise with negative X
            // (left)
            ));
  }
  // Set up auto routines
  // autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

  // Set up SysId routines
  // autoChooser.addOption(
  //     "Drive Wheel Radius Characterization",
  // DriveCommands.wheelRadiusCharacterization(drive));
  // autoChooser.addOption(
  //     "Drive Simple FF Characterization",
  // DriveCommands.feedforwardCharacterization(drive));
  // autoChooser.addOption(
  //     "Drive SysId (Quasistatic Forward)",
  //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
  // autoChooser.addOption(
  //     "Drive SysId (Quasistatic Reverse)",
  //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
  // autoChooser.addOption(
  //     "Drive SysId (Dynamic Forward)",
  // drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
  // autoChooser.addOption(
  //     "Drive SysId (Dynamic Reverse)",
  // drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

  // Configure the button bindings
  //   configureButtonBindings();
  //   assembleLightsSuppliers();
  // }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // operatorController.getA().onTrue(Commands.run(() -> elevator.setPosition(10)));
    // operatorController.getY().onTrue(Commands.run(() -> elevator.setPosition(0)));
    // operatorController.getDPadDownLeft().onTrue(Commands.run(() -> arm.setPosition(0)));
    // operatorController.getB().onTrue(Commands.run(() -> arm.setPosition(-10)));
    // operatorController.getDPadLeft().onTrue(gripper.setVoltage(-5));
    operatorController.getDPadRight().onTrue(superstructure.goToLevel(Position.AlgaeHome));
    operatorController.getDPadLeft().onTrue(superstructure.goToLevel(Position.AlgaePickup));
    // operatorController.getY().onTrue(intake.goToPositionCommand(-25));
    // operatorController.getX().onTrue(intake.setTargetPosition(0));
    // operatorController.getDPadDown().onTrue(intake.setWheelsVoltage(-5));
    // operatorController.getDPadUp().onTrue(straightenator.runBothWheelsCorrect(4));
    // Command cradleCommand =
    //     Commands.sequence(
    //         elevator.goToPositionCommand(0),
    //         Commands.parallel(elevator.goToPositionCommand(0), arm.goToPositionCommand(0.44)));
    // // gripper.setVoltage(8).withTimeout(0.7));

    // operatorController.getA().onTrue(cradleCommand);

    // Command placeCommand =
    //     Commands.sequence(
    //         Commands.parallel(elevator.goToPositionCommand(5)),
    //         Commands.parallel(arm.goToPositionCommand(0.4)),
    //         Commands.parallel(elevator.goToPositionCommand(5)),
    //         gripper.setVoltage(8).withTimeout(0.7));

    // operatorController.getB().onTrue(placeCommand);

    // operatorController.getDPadUp().onTrue(climber.setVoltage(4));

    //     rightJoystick
    //         .getLeftTopLeft()
    //         .onTrue(
    //             Commands.runOnce(
    //                 () ->
    //                     drivetrain.resetPose(
    //                         new Pose2d(0, 0, drivetrain.getOperatorForwardDirection()))));

    rightJoystick
        .getTrigger()
        .onTrue(Commands.defer(() -> superstructure.execute(), Set.of(superstructure)));

    rightJoystick
        .getLeftThumb()
        .whileTrue(
            superstructure.intakeToCradle(
                roller, intake, straightenator)); // roller, intake, straightenator
    //     leftJoystick
    //         .getLeftThumb()
    //         .whileTrue(superstructure.intakeAlgae(Position.AlgaePickup)); // TODO: Beam break DIO
    // 6

    operatorController
        .getA()
        // .and(() -> superstructure.getCurrentScoringMode() == ScoringMode.Coral)
        .onTrue(superstructure.goToLevel(Position.L1));
    operatorController
        .getB()
        // .and(() -> superstructure.getCurrentScoringMode() == ScoringMode.Coral)
        .onTrue(superstructure.goToLevel(Position.L2Prep));
    operatorController
        .getX()
        // .and(() -> superstructure.getCurrentScoringMode() == ScoringMode.Coral)
        .onTrue(superstructure.goToLevel(Position.L3Prep));
    operatorController
        .getY()
        // .and(() -> superstructure.getCurrentScoringMode() == ScoringMode.Coral)
        .onTrue(superstructure.goToLevel(Position.L4Prep));

    //     operatorController.getStart().onTrue(superstructure.goToLevel(Position.ClimbPosition));

    operatorController.getDPadUp().onTrue(superstructure.goToLevel(Position.CoralHome));

    operatorController.getDPadDown().onTrue(superstructure.intakeCoral());

    // operatorController
    //     .getY()
    //     .onTrue(
    //         Commands.defer(
    //             () -> superstructure.goToLevel(Position.L4),
    //             Set.of(superstructure, elevator, arm, gripper, intake, roller, straightenator)));
    // operatorController
    //     .getX()
    //     .onTrue(
    //         Commands.defer(
    //             () -> superstructure.goToLevel(Position.L3),
    //             Set.of(superstructure, elevator, arm, gripper, intake, roller, straightenator)));
    // operatorController
    //     .getB()
    //     .onTrue(
    //         Commands.defer(
    //             () -> superstructure.goToLevel(Position.L2),
    //             Set.of(superstructure, elevator, arm, gripper, intake, roller, straightenator)));
    // operatorController
    //     .getA()
    //     .onTrue(
    //         Commands.defer(
    //             () -> superstructure.goToLevel(Position.L1),
    //             Set.of(superstructure, elevator, arm, gripper, intake, roller, straightenator)));

    operatorController.getLeftBumper().onTrue(superstructure.intakeAlgae(Position.AlgaePickup));
    // leftJoystick
    //     .getBottomThumb()
    //     .and(() -> superstructure.getCurrentScoringMode() == ScoringMode.Coral)
    //     .whileTrue(alignToReef(AlignConstants.leftOffset));
    //     rightJoystick
    //         .getBottomThumb()
    //         .and(() -> superstructure.getCurrentScoringMode() == ScoringMode.Coral)
    //         .whileTrue(alignToReef(AlignConstants.rightOffset));
    //     rightJoystick
    //         .getBottomThumb()
    //         .and(() -> superstructure.getCurrentScoringMode() == ScoringMode.Algae)
    //         .whileTrue(alignToReef(AlignConstants.centerOffset));

    //     operatorController
    //         .getY()
    //         .and(() -> superstructure.getCurrentScoringMode() == ScoringMode.Algae)
    //         .onTrue(superstructure.goToLevel(Position.AlgaeNetFacing));
    // operatorController
    //     .getA()
    //     .and(() -> superstructure.getCurrentScoringMode() == ScoringMode.Algae)
    //     .onTrue(superstructure.goToLevel(Position.AlgaeProcessor));
    //     operatorController
    //         .getB()
    //         .and(() -> superstructure.getCurrentScoringMode() == ScoringMode.Algae)
    //         .onTrue(superstructure.intakeAlgae(Position.AlgaeL2));
    //     operatorController
    //         .getX()
    //         .and(() -> superstructure.getCurrentScoringMode() == ScoringMode.Algae)
    //         .onTrue(superstructure.intakeAlgae(Position.AlgaeL3));

    // Default command, normal field-relative drive

    //   drivetrain.setDefaultCommand(
    //       // Drivetrain will execute this command periodically
    //       drivetrain.applyRequest(
    //           () ->
    //               drive
    //                   .withVelocityY(
    //                       -Math.pow(leftJoystick.getXAxis().getRaw(), 3)
    //                           * MaxSpeed) // Drive forward with negative Y
    //                   // (forward) POSSIBLY READD -
    //                   // TO FIX ANY INVERT ISSUES
    //                   .withVelocityX(
    //                       -Math.pow(leftJoystick.getYAxis().getRaw(), 3)
    //                           * MaxSpeed) // Drive left with negative X
    //                   // (left)
    //                   .withRotationalRate(
    //                       Math.pow(-rightJoystick.getXAxis().getRaw(), 3) * MaxAngularRate)
    //                   .withDeadband(0.02) // Drive counterclockwise with negative X
    //           // (left)
    //           ));
  }

  private void assembleLightsSuppliers() {
    LightsControlModule.Supplier_hasPiece(() -> gripper.hasPiece());
    LightsControlModule.Supplier_isAligning(rightJoystick.getBottomThumb());
    LightsControlModule.Supplier_alignMode(() -> superstructure.getCurrentScoringMode().ordinal());
    //   LightsControlModule.Supplier_batteryVoltage(() -> RobotController.getBatteryVoltage());
    LightsControlModule.Supplier_opControllerLeftX(() -> operatorController.getLeftXAxis().get());
    LightsControlModule.Supplier_opControllerLeftY(() -> operatorController.getLeftYAxis().get());
    LightsControlModule.Supplier_opControllerRightX(() -> operatorController.getRightXAxis().get());
    LightsControlModule.Supplier_opControllerRightY(() -> operatorController.getRightYAxis().get());
  }

  public Command alignToReef(int tag, double offset, Rotation2d rotOffset) {
    Pose2d alignmentPose =
        VisionConstants.aprilTagLayout
            .getTagPose(tag)
            .get()
            .toPose2d()
            .plus(
                new Transform2d(new Translation2d(AlignConstants.reefDistance, offset), rotOffset));
    return new AlignToReef(drivetrain, offset, alignmentPose, Rotation2d.kPi);
  }

  public Command alignToReef(int tag, double offset) {
    return alignToReef(tag, offset, Rotation2d.kZero);
  }

  public Command alignToReef(double offset) {
    return Commands.defer(
        () -> {
          Pose2d alignmentPose = drivetrain.findNearestAprilTagPose();
          return new AlignToReef(drivetrain, offset, alignmentPose, Rotation2d.kPi);
        },
        Set.of(drivetrain));
  }

  public Command alignAndDriveToReef(int tag, double offset) {
    Pose2d alignmentPose =
        VisionConstants.aprilTagLayout
            .getTagPose(tag)
            .get()
            .toPose2d()
            .plus(
                new Transform2d(
                    new Translation2d(AlignConstants.reefDistance, offset), new Rotation2d()));
    return new AlignAndDriveToReef(drivetrain, 0, alignmentPose, Rotation2d.kPi);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
