package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.gripper.GripperSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.straightenator.StraightenatorSubsystem;

import javax.swing.text.Position;

public class Superstructure extends SubsystemBase {
  private ElevatorSubsystem elevator;
  private ArmSubsystem arm;
  private IntakeSubsystem intake;
  private StraightenatorSubsystem straightenator;
  private GripperSubsystem gripper;
  private Position targetPosition = Position.Pick;
  private Position currentPosition = Position.Pick;
  public Position lastPosition = Position.Pick;

  public Superstructure(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, GripperSubsystem gripper) {
    this.elevator = elevator;
    this.arm = arm;
    this.gripper = gripper;
  }

  enum Position {
    Empty(0, 0, null),
    AlgaeHome(0, 0, Empty),
    CoralHome(0, 0, Empty),
    Pick(0, 0, CoralHome),
    L4(0, 0, CoralHome),
    L3(0, 0, CoralHome),
    L2(0, 0, CoralHome),
    L1(0, 0, CoralHome),
    L4Prep(0, 0, L4),
    L3Prep(0, 0, L3),
    L2Prep(0, 0, L2),
    AlgaeL2(0, 0, Empty),
    AlgaeL3(0, 0, Empty),
    AlgaeNet(0, 0, Empty),
    AlgaeProcesser(0, 0, Empty),
    AlgaePickup(0, 0, AlgaeHome),
    ClimbPosition(0, 0, Empty),
    SuperstructurePosition(0, 0, Empty);

    private double elevatorHeight;
    private double armAngle;
    private Position nextPosition;

    private Position(double elevatorHeight, double armAngle, Position nextPosition) {
      this.elevatorHeight = elevatorHeight;
      this.armAngle = armAngle;
      this.nextPosition = nextPosition;
    }

    public double elevatorHeight() {
      return elevatorHeight;
    }

    public double armAngle() {
      return armAngle;
    }

    public Position nextPosition() {
      return nextPosition;
    }
  }

  public static enum Align {
    zero,
    leftAlign,
    rightAlign,
    algaeAlign;
  }

  private Command moveArm(Position position) {
    return Commands.runOnce(() -> arm.setPosition(position.armAngle))
        .andThen(Commands.waitUntil(() -> arm.isAtSetpoint()));
  }

  public Command moveElevator(Position position) {
    return Commands.runOnce(() -> elevator.setPosition(position.elevatorHeight));
  }

  public Command goToPrep(Position Position) {

    return Commands.runOnce(() -> targetPosition = Position, this)
        .andThen(
            Commands.either(
                Commands.sequence(
                    moveElevator(Position),
                    Commands.waitUntil(
                        () -> elevator.isAtSetpoint()),
                    moveArm(Position),
                    Commands.waitUntil(() -> arm.isAtSetpoint())),
                Commands.sequence(
                    moveArm(Position),
                    Commands.waitUntil(() -> arm.isAtSetpoint()),
                    moveElevator(Position),
                    Commands.waitUntil(
                        () -> elevator.isAtSetpoint())),
                () -> lastPosition == Position.Pick))
        .andThen(Commands.runOnce(() -> lastPosition = Position, this));
  }

  // TODO: perhaps remove this later
  // TODO: alex, study DRY
  public Command goToLevel(Position Position) {

    return Commands.runOnce(() -> targetPosition = Position, this)
        .andThen(
            Commands.either(
                Commands.sequence(
                    moveElevator(Position),
                    Commands.waitUntil(
                        () -> elevator.isAtSetpoint()),
                    moveArm(Position),
                    Commands.waitUntil(() -> arm.isAtSetpoint())),
                Commands.sequence(
                    moveArm(Position),
                    Commands.waitUntil(() -> arm.isAtSetpoint()),
                    moveElevator(Position),
                    Commands.waitUntil(
                        () -> elevator.isAtSetpoint())),
                () -> lastPosition == Position.Pick))
        .andThen(Commands.runOnce(() -> lastPosition = Position, this));
  }

  public Command goToHome(Position Position) {

    return Commands.runOnce(() -> targetPosition = Position, this)
        .andThen(
            Commands.either(
                Commands.sequence(
                    moveElevator(Position),
                    Commands.waitUntil(
                        () -> elevator.isAtSetpoint()),
                    moveArm(Position),
                    Commands.waitUntil(() -> arm.isAtSetpoint())),
                Commands.sequence(
                    moveArm(Position),
                    Commands.waitUntil(() -> arm.isAtSetpoint()),
                    moveElevator(Position),
                    Commands.waitUntil(
                        () -> elevator.isAtSetpoint())),
                () -> lastPosition == Position.Pick))
        .andThen(Commands.runOnce(() -> lastPosition = Position, this));
  }

  // TODO: real intake pivot positions
  public Command intakeToCradle() {
    Command runIntake = Commands.parallel(intake.setTargetPosition(20), 
                                          intake.setWheelsVoltage(12), 
                                          straightenator.setWheelVoltage(12));

    return runIntake.until(() -> straightenator.isCradled()).andThen(intake.setTargetPosition(0));
  }


  public Command scoreCoral(Position prepPosition, Position position) {
    return Commands.sequence(goToLevel(prepPosition), goToLevel(position), gripper.placePiece(), goToHome(Position.CoralHome));
  }

  public Command intakeAlgae(Position position) {
    return Commands.sequence(goToLevel(position), gripper.intakeUntilPieceDetected(), goToHome(Position.AlgaeHome));
  }


  



  
  

}
