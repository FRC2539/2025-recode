// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.GripperConstants;
// import org.littletonrobotics.junction.Logger;
// import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.littletonrobotics.junction.Logger;

public class GripperSubsystem extends SubsystemBase {

  private GripperIO gripperIO;
  private GripperIOInputsAutoLogged gripperInputs = new GripperIOInputsAutoLogged();
  private final Trigger HAS_PIECE = new Trigger(this::hasPiece);

  public GripperSubsystem(GripperIO gripperIO) {
    this.gripperIO = gripperIO;
    setDefaultCommand(setVoltage(-2));

    // Command idleCommand = setVoltage(-0.5);
    // Command stopCommand = setVoltage(0);

    // Command conditionalDefault = Commands.either(idleCommand, stopCommand, this::hasPiece);

    // setDefaultCommand(conditionalDefault);
  }

  @Override
  public void periodic() {
    gripperIO.updateInputs(gripperInputs);
    Logger.processInputs("RealOutputs/Gripper", gripperInputs);
  }

  public Command placePiece() {
    return Commands.race(
        setVoltage(GripperConstants.gripperPlacementVoltage), Commands.waitSeconds(0.5));
  }

  public Command placePieceL1() {
    return Commands.race(
        setVoltage(GripperConstants.gripperPlacementVoltageL1), Commands.waitSeconds(0.5));
    // return setVoltage(GripperConstants.gripperPlacementVoltageL1).until(HAS_PIECE.negate());
  }

  public Command intakeUntilPieceDetected() {

    return Commands.run(
        () -> {
          setVoltage(GripperConstants.intakeVoltage).until(() -> hasPiece());
        },
        this);
  }

  public Command setVoltage(double voltage) {
    return Commands.run(
        () -> {
          // System.out.println("voltage: " + voltage);
          gripperIO.setVoltage(voltage);
        },
        this);
  }

  public Command setReverse(double voltage) {
    return Commands.run(
        () -> {
          gripperIO.setVoltage(-voltage);
        },
        this);
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
