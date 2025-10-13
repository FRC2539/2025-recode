package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.GripperConstants.Piece;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.gripper.GripperSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.roller.RollerSubsystem;
import frc.robot.subsystems.straightenator.StraightenatorSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;

public class Superstructure extends SubsystemBase {
  private ElevatorSubsystem elevator;
  private ArmSubsystem arm;
  private IntakeSubsystem intake;
  private StraightenatorSubsystem straightenator;
  private GripperSubsystem gripper;
  private RollerSubsystem roller;
  @AutoLogOutput public Position targetPosition = Position.Pick;
  @AutoLogOutput public Position currentPosition = Position.Pick;
  public Position lastPosition = Position.Pick;

  public ScoringMode currentScoringMode = ScoringMode.None;

  public Superstructure(
      ElevatorSubsystem elevatorSubsystem,
      ArmSubsystem armSubsystem,
      GripperSubsystem gripper,
      IntakeSubsystem intake,
      RollerSubsystem roller,
      StraightenatorSubsystem straightenator) {
    this.elevator = elevatorSubsystem;
    this.arm = armSubsystem;
    this.gripper = gripper;
    this.intake = intake;
    this.straightenator = straightenator;
    this.roller = roller;
  }

  public static enum Position {
    AlgaeHome(3.9, -0.16), //
    CoralHome(10, .272),
    Pick(-0.25, .331),
    L4(37.5, 0.1),
    L3(16.137, 0.086),
    L2(0.964, 0.072),
    L1(7.04, .123),
    L4Prep(36.5, -0.04),
    shawn(41, -0.04),
    L3Prep(16.137, -0.05),
    L2Prep(0.964, -0.05),
    AlgaeL2(11, 0.07),
    AlgaeL3(25.3, 0.08),
    // AlgaeNetFacing(43, .75),
    AlgaeNetPrep(43, -0.22),
    AlgaeNet(43, -0.22), //
    AlgaeProcessor(.8, 0.12),
    AlgaePickup(-0.6, .205),
    ClimbPosition(0, 0),
    SuperstructurePosition(2.647, 0.308);

    private double elevatorHeight;
    private double armAngle;

    private Position(double elevatorHeight, double armAngle) {
      this.elevatorHeight = elevatorHeight;
      this.armAngle = armAngle;
    }

    public double elevatorHeight() {
      return elevatorHeight;
    }

    public double armAngle() {
      return armAngle;
    }
  }

  public static enum ScoringMode {
    Algae,
    Coral,
    None;
  }

  public static enum Align {
    zero,
    leftAlign,
    rightAlign;
  }

  public static enum Height {
    L4,
    L2and3,
    Algae;
  }

  public void setScoringMode(ScoringMode mode) {
    this.currentScoringMode = mode;
  }

  public ScoringMode getCurrentScoringMode() {
    return this.currentScoringMode;
  }

  private Command moveArm(Position position) {
    return Commands.runOnce(() -> arm.setPosition(position.armAngle))
        .andThen(Commands.waitUntil(() -> arm.isAtSetpoint()));
  }

  public Command moveElevator(Position position) {
    return Commands.runOnce(() -> elevator.setPosition(position.elevatorHeight))
        .andThen(Commands.waitUntil(() -> elevator.isAtSetpoint()));
  }

  //   public Command goToLevel(Position position) {
  //     return Commands.runOnce(() -> targetPosition = position, this)
  //         .andThen(
  //             Commands.either(
  //                 Commands.sequence(
  //                     moveElevator(position),
  //                     Commands.waitUntil(() -> elevator.isAtSetpoint()),
  //                     moveArm(position),
  //                     Commands.waitUntil(() -> arm.isAtSetpoint())),
  //                 Commands.sequence(
  //                     moveArm(position),
  //                     Commands.waitUntil(() -> arm.isAtSetpoint()),
  //                     moveElevator(position),
  //                     Commands.waitUntil(() -> elevator.isAtSetpoint())),
  //                 () -> lastPosition == Position.Pick))
  //         .andThen(Commands.runOnce(() -> lastPosition = position, this));
  //   }

  public Command goToLevel(Position position) {
    Command elevatorThenArm = Commands.sequence(moveElevator(position), moveArm(position));

    Command armThenElevator = Commands.sequence(moveArm(position), moveElevator(position));

    return Commands.runOnce(() -> targetPosition = position, this)
        .andThen(
            Commands.either(
                Commands.sequence(moveElevator(position), moveArm(position)),
                Commands.sequence(moveArm(position), moveElevator(position)),
                () -> lastPosition == Position.Pick || targetPosition == Position.Pick))
        .andThen(Commands.runOnce(() -> lastPosition = position, this));
  }

  public Command goToLevelFast(Position position) {
    return Commands.runOnce(() -> targetPosition = position, this)
        .andThen(Commands.parallel(moveElevator(position), moveArm(position)))
        .andThen(Commands.runOnce(() -> lastPosition = position, this));
  }

  public Command goToLevelNet(Position position) {
    return Commands.runOnce(() -> targetPosition = position, this)
        .andThen(Commands.sequence(moveElevator(position), moveArm(position)))
        .andThen(Commands.runOnce(() -> lastPosition = position, this))
        .andThen(gripper.setVoltage(-1));
  }

  //   public Command goToLevelpick(Position position) {
  //     return Commands.runOnce(() -> targetPosition = position, this)
  //         .andThen(
  //             Commands.either(
  //                 Commands.sequence(
  //                     moveElevator(position),
  //                     Commands.waitUntil(() -> elevator.isAtSetpoint()),
  //                     moveArm(position),
  //                     Commands.waitUntil(() -> arm.isAtSetpoint())),
  //                 Commands.sequence(
  //                     moveElevator(position),
  //                     Commands.waitUntil(() -> elevator.isAtSetpoint()),
  //                     moveArm(position),
  //                     Commands.waitUntil(() -> arm.isAtSetpoint())),
  //                 () -> lastPosition == Position.Pick))
  //         .andThen(Commands.runOnce(() -> lastPosition = position, this));
  //   }

  public Command goToLevelpick() {
    Command moveSequence = Commands.sequence(moveArm(Position.Pick), moveElevator(Position.Pick));

    return Commands.runOnce(() -> targetPosition = Position.Pick, this)
        .andThen(moveSequence)
        .andThen(Commands.runOnce(() -> lastPosition = Position.Pick, this));
  }

  public Command intakeToCradle(
      RollerSubsystem roller, IntakeSubsystem intake, StraightenatorSubsystem straightenator) {
    Command runIntake =
        Commands.parallel(
            intake.goToPositionCommand(IntakeConstants.intakeDownPosition),
            roller.setWheelsVoltage(-8),
            straightenator.runBothWheelsCorrect(5));

    return runIntake
        .until(() -> straightenator.isCradled())
        .andThen(intake.setTargetPosition(IntakeConstants.intakeUpPosition));
  }

  public Command intakeToCradleAuto(
      RollerSubsystem roller, IntakeSubsystem intake, StraightenatorSubsystem straightenator) {
    Command runIntake =
        Commands.parallel(
            intake.goToPositionCommand(IntakeConstants.intakeDownPosition),
            roller.setWheelsVoltage(-8),
            straightenator.runBothWheelsCorrect(5));

    // return runIntake
    //     .until(() -> straightenator.isCradled())
    //     .andThen(intake.setTargetPosition(IntakeConstants.intakeUpPosition).withTimeout(5));

    Command intakeUntilCradled =
        runIntake
            .until(() -> straightenator.isCradled())
            .andThen(intake.setTargetPosition(IntakeConstants.intakeUpPosition));

    return intakeUntilCradled;
  }

  public Command extake(
      RollerSubsystem roller, IntakeSubsystem intake, StraightenatorSubsystem straightenator) {
    Command runIntake =
        Commands.sequence(
            intake.goToPositionCommand(IntakeConstants.intakeDownPosition),
            Commands.parallel(
                roller.setWheelsVoltage(5), straightenator.runBothWheelsBackwards(5)));

    return runIntake
        // .until(() -> straightenator.isCradled())
        .andThen(intake.setTargetPosition(IntakeConstants.intakeUpPosition));
  }

  public Command scoreCoral(Position position) {
    return Commands.sequence(
        Commands.parallel(goToLevel(position), gripper.placePiece()),
        goToLevel(Position.CoralHome));
  }

  public Command scoreCoralAuto(Position position) {
    return Commands.deadline(gripper.placePiece(), goToLevel(position));
  }

  public Command scoreAlgae(Position position) {
    return Commands.sequence(gripper.placePieceAlgae());
    // goToLevel(Position.CoralHome));
  }

  public Command execute() {
    switch (targetPosition) {
      case L4Prep:
        // return gripper.placePiece();
        return scoreCoral(Position.L4);
      case shawn:
        return scoreCoral(Position.L4);
      case L3Prep:
        return scoreCoral(Position.L3);
      case L2Prep:
        return scoreCoral(Position.L2);
      case L1:
        return gripper.placePieceL1();

      default:
        return gripper.placePieceAlgae();
        // return Commands.none();
    }
  }

  public Command executeAuto() {

    // if (!gripper.hasPiece()) {
    //   return Commands.none();
    // }
    switch (targetPosition) {
      case L4Prep:
        // return gripper.placePiece();
        return scoreCoralAuto(Position.L4);
      case shawn:
        return scoreCoralAuto(Position.L4);
      case L3Prep:
        return scoreCoralAuto(Position.L3);
      case L2Prep:
        return scoreCoralAuto(Position.L2);
      case L1:
        return gripper.placePieceL1();

      default:
        return gripper.placePieceAlgae();
        // return Commands.none();
    }
  }

  public Command intakeAlgae(Position position) {
    return Commands.sequence(
        goToLevel(position),
        gripper.intakeUntilPieceDetected(),
        // goToLevel(Position.AlgaeHome),
        gripper.setVoltage(-1.5));
  }

  public Command intakeCoral() {
    if (currentPosition == Position.CoralHome || targetPosition == Position.CoralHome) {
      return Commands.sequence(
          Commands.parallel(goToLevelpick(), gripper.intakeUntilPieceDetected()),
          goToLevel(Position.SuperstructurePosition),
          Commands.waitSeconds(0.25),
          goToLevel(Position.CoralHome));
    }
    return Commands.none();
  }

  public Command intakeCoralAuto() {
    return Commands.sequence(
        Commands.parallel(goToLevelpick(), gripper.intakeUntilPieceDetected()),
        goToLevel(Position.SuperstructurePosition),
        Commands.waitSeconds(0.25),
        goToLevel(Position.CoralHome));
  }

  public Command updateTargetPosition(Position position) {
    return Commands.runOnce(() -> targetPosition = position, this);
  }

  @Override
  public void periodic() {
    Piece currentPiece = gripper.getPieceType();
    if (currentPiece == Piece.CORAL) {
      setScoringMode(ScoringMode.Coral);
    } else if (currentPiece == Piece.ALGAE) {
      setScoringMode(ScoringMode.Algae);
    } else {
      setScoringMode(ScoringMode.None);
    }
  }
}
