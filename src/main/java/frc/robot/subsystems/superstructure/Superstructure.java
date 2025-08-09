package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.GripperConstants.Piece;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.gripper.GripperSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.straightenator.StraightenatorSubsystem;

public class Superstructure extends SubsystemBase {
  private ElevatorSubsystem elevator;
  private ArmSubsystem arm;
  private IntakeSubsystem intake;
  private StraightenatorSubsystem straightenator;
  private GripperSubsystem gripper;
  public Position targetPosition = Position.Pick;
  public Position currentPosition = Position.Pick;
  public Position lastPosition = Position.Pick;

  public ScoringMode currentScoringMode = ScoringMode.None;

  public Superstructure(
      ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, GripperSubsystem gripper) {
    this.elevator = elevatorSubsystem;
    this.arm = armSubsystem;
    this.gripper = gripper;
  }

  public static enum Position {
    AlgaeHome(0, 0),
    CoralHome(0, 0),
    Pick(0, 0),
    L4(0, 0),
    L3(0, 0),
    L2(0, 0),
    L1(0, 0),
    L4Prep(0, 0),
    L3Prep(0, 0),
    L2Prep(0, 0),
    AlgaeL2(0, 0),
    AlgaeL3(0, 0),
    AlgaeNet(0, 0),
    AlgaeProcessor(0, 0),
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
    return Commands.runOnce(() -> elevator.setPosition(position.elevatorHeight));
  }

  public Command goToLevel(Position Position) {
    return Commands.runOnce(() -> targetPosition = Position, this)
        .andThen(
            Commands.either(
                Commands.sequence(
                    moveElevator(Position),
                    Commands.waitUntil(() -> elevator.isAtSetpoint()),
                    moveArm(Position),
                    Commands.waitUntil(() -> arm.isAtSetpoint())),
                Commands.sequence(
                    moveArm(Position),
                    Commands.waitUntil(() -> arm.isAtSetpoint()),
                    moveElevator(Position),
                    Commands.waitUntil(() -> elevator.isAtSetpoint())),
                () -> lastPosition == Position.Pick))
        .andThen(Commands.runOnce(() -> lastPosition = Position, this));
  }

  // TODO: real intake pivot positions
  public Command intakeToCradle() {
    Command runIntake =
        Commands.parallel(
            intake.setTargetPosition(20),
            intake.setWheelsVoltage(12),
            straightenator.setWheelVoltage(12));

    return runIntake.until(() -> straightenator.isCradled()).andThen(intake.setTargetPosition(0));
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

  public Command updateTargetPosition(Position position) {
    return Commands.runOnce(() -> targetPosition = position, this);
  }

  @Override
  public void periodic() {
    if (gripper.getPieceType() == Piece.CORAL) {
      setScoringMode(ScoringMode.Coral);
    } else if (gripper.getPieceType() == Piece.ALGAE) {
      setScoringMode(ScoringMode.Algae);
    } else {
      setScoringMode(ScoringMode.None);
    }
  }
}
