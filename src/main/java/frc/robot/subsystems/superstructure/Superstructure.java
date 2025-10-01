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

public class Superstructure extends SubsystemBase {
  private ElevatorSubsystem elevator;
  private ArmSubsystem arm;
  private IntakeSubsystem intake;
  private StraightenatorSubsystem straightenator;
  private GripperSubsystem gripper;
  private RollerSubsystem roller;
  public Position targetPosition = Position.Pick;
  public Position currentPosition = Position.Pick;
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
    AlgaeHome(0, 0),
    CoralHome(10, -6),
    Pick(0, 0),
    L4(16, -11),
    L3(14, -11),
    L2(12, -11),
    L1(10, -11),
    L4Prep(16, -15),
    L3Prep(14, -15),
    L2Prep(12, -15),
    AlgaeL2(10, -15),
    AlgaeL3(10, -15),
    AlgaeNetFacing(10, -15),
    AlgaeNetLimelight(10, -15),
    AlgaeProcessor(10, -15),
    AlgaePickup(0, 0),
    ClimbPosition(0, 0),
    SuperstructurePosition(0, 0);

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
    Command elevatorThenArm = Commands.sequence(
        moveElevator(position),
        moveArm(position));

    Command armThenElevator = Commands.sequence(
        moveArm(position),
        moveElevator(position));

    return Commands.runOnce(() -> targetPosition = position, this)
        .andThen(
            Commands.either(
                elevatorThenArm,
                armThenElevator,
                () -> lastPosition == Position.Pick))
        .andThen(Commands.runOnce(() -> lastPosition = position, this));
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

public Command goToLevelpick(Position position) {
    // This is the core movement logic, defined once.
    Command moveSequence = Commands.sequence(
        moveArm(position),
        moveElevator(position)
    );

    return Commands.runOnce(() -> targetPosition = position, this)
        .andThen(moveSequence)
        .andThen(Commands.runOnce(() -> lastPosition = position, this));
}

  public Command intakeToCradle(
      RollerSubsystem roller, IntakeSubsystem intake, StraightenatorSubsystem straightenator) {
    Command runIntake =
        Commands.parallel(
            intake.goToPositionCommand(IntakeConstants.intakeDownPosition),
            roller.setWheelsVoltage(-5),
            straightenator.runBothWheelsCorrect(5));

    return runIntake
        .andThen(
            Commands.waitUntil(
                () ->
                    straightenator
                        .isCradled())) // .andThen(Commands.waitUntil(straightenator.isCradled()));
        .andThen(intake.setTargetPosition(IntakeConstants.intakeUpPosition));
  }

  public Command scoreCoral(Position prepPosition, Position position) {
    return Commands.sequence(
        goToLevel(prepPosition),
        goToLevel(position),
        gripper.placePiece(),
        goToLevel(Position.CoralHome));
  }

  public Command execute() {
    switch (targetPosition) {
      case L4:
        return scoreCoral(Position.L4Prep, Position.L4);
      case L3:
        return scoreCoral(Position.L3Prep, Position.L3);
      case L2:
        return scoreCoral(Position.L2Prep, Position.L2);
      case L1:
        return gripper.placePiece();

      default:
        return Commands.none();
    }
  }

  public Command intakeAlgae(Position position) {
    return Commands.sequence(
        goToLevel(position), gripper.intakeUntilPieceDetected(), goToLevel(Position.AlgaeHome));
  }

  public Command intakeCoral() {
    return Commands.sequence(
        Commands.parallel(goToLevelpick(Position.Pick), gripper.intakeUntilPieceDetected()),
        goToLevel(Position.CoralHome));
  }

  public Command updateTargetPosition(Position position) {
    return Commands.runOnce(() -> targetPosition = position, this);
  }

  @Override
  public void periodic() {
    // if (gripper.getPieceType() == Piece.CORAL) {
    //   setScoringMode(ScoringMode.Coral);
    // } else if (gripper.getPieceType() == Piece.ALGAE) {
    //   setScoringMode(ScoringMode.Algae);
    // } else {
    //   setScoringMode(ScoringMode.None);
    // }
  }
}
