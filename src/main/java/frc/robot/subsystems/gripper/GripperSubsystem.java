// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.GripperConstants;
import frc.robot.subsystems.gripper.GripperIO.GripperIOInputs;
// import org.littletonrobotics.junction.Logger;
// import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class GripperSubsystem extends SubsystemBase {

  private GripperIO gripperIO;
  private GripperIOInputs gripperInputs = new GripperIOInputs();
  private final Trigger HAS_PIECE = new Trigger(this::hasPiece);

  public GripperSubsystem(GripperIO gripperIO) {
    this.gripperIO = gripperIO;
    setDefaultCommand(setVoltage(0));
  }

  @Override
  public void periodic() {
    gripperIO.updateInputs(gripperInputs);
    // Logger.processInputs("RealOutputs/Gripper", gripperInputs);
  }

  public Command placePiece() {
    return setVoltage(GripperConstants.gripperPlacementVoltage).until(HAS_PIECE.negate());
  }

  public Command intakeUntilPieceDetected() {

    return Commands.run(
        () -> {
          setVoltage(GripperConstants.intakeVoltage).until(() -> hasPiece());
        });
  }

  public Command setVoltage(double voltage) {
    return Commands.run(
        () -> {
          gripperIO.setVoltage(voltage);
        });
  }

  public Command setReverse(double voltage) {
    return Commands.run(
        () -> {
          gripperIO.setVoltage(-voltage);
        });
  }

  public boolean hasPiece() {
    return gripperInputs.hasPiece;
  }

  public boolean intaking() {
    return gripperInputs.voltage > 1;
  }

  public GripperConstants.Piece getPieceType() {
    return gripperInputs.pieceType;
  }
}
