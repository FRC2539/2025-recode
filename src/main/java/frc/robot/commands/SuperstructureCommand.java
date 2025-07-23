package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.superstructure.Superstructure;

public class SuperstructureCommand extends Command {

  private final ArmSubsystem armSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final Superstructure superstructure;

  public SuperstructureCommand(
      ArmSubsystem armSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      Superstructure superstructure) {
    this.superstructure = superstructure;
    this.armSubsystem = armSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
