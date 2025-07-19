package frc.robot.subsystems.superstructure;

import javax.swing.text.Position;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class Superstructure extends SubsystemBase{
    private ElevatorSubsystem elevatorSubsystem;
    private ArmSubsystem armSubsystem;
    private Position targetPosition = Position.Pick;

    public Superstructure(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.armSubsystem = armSubsystem;
    }

    enum Position {
        Empty(0, 0, null),
        AlgaeHome(0, 0, Empty),
        CoralHome(0, 0, Empty),
        Pick(0,0, Empty),
        L4(0, 0, CoralHome),
        L3(0, 0, CoralHome),
        L2(0, 0, CoralHome),
        L1(0, 0, CoralHome),
        L4Prep(0, 0, L4),
        L3Prep(0, 0, L3),
        L2Prep(0,0, L2),
        AlgaeL2(0,0, Empty),
        AlgaeL3(0,0, Empty),
        AlgaeNet(0,0, Empty),
        AlgaeProcesser(0,0, Empty),
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
        return Commands.runOnce(() -> armSubsystem.setPosition(position.armAngle)).andThen(Commands.waitUntil(() -> armSubsystem.isAtSetpoint()));
    } 

    public Command moveElevator(Position position) {
        return Commands.runOnce(() -> elevatorSubsystem.setPosition(position.elevatorHeight));
    }

    public Command execute() {
        return Commands.runOnce(() -> {});
    }

    // public Command goTo(Position targetPosition) {
    //     return Commands.either(Commands.parallel(moveArm(targetPosition), moveElevator(targetPosition)), 
    //                            Commands.sequence(moveElevator(targetPosition), moveArm(targetPosition)), 
    //                            () -> elevatorSubsystem.getPosition() < 20); // TODO: WHAT IS THIS POSITION
    
    // }

}
