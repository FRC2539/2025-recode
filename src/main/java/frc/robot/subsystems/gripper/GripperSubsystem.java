// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.GripperConstants;
import frc.robot.subsystems.lights.LightsSubsystem;
import org.littletonrobotics.junction.Logger;

public class GripperSubsystem extends SubsystemBase {

  private GripperIO gripperIO;
  private GripperIOInputsAutoLogged gripperInputs = new GripperIOInputsAutoLogged();
  private final Trigger HAS_PIECE = new Trigger(this::hasPiece);

  private static final double ALGAE_IDLE_VOLTAGE = -2;
  private static final double DEFAULT_IDLE_VOLTAGE = -0.6;

  private final LightsSubsystem lights;
  public static final RGBWColor orange = new RGBWColor(230, 25, 0);
  public static final RGBWColor pink = new RGBWColor(230, 0, 100);
  public static final RGBWColor green = new RGBWColor(0, 255, 0);

  public GripperSubsystem(GripperIO gripperIO, LightsSubsystem lights) { // <-- Pass Lights in
    this.gripperIO = gripperIO;
    this.lights = lights; // <-- Store the reference
    setDefaultCommand(setVoltage(-2));

    // public GripperSubsystem(GripperIO gripperIO) {
    //   this.gripperIO = gripperIO;
    //   setDefaultCommand(setVoltage(-2));

    Command dynamicIdleCommand =
        Commands.run(
                () -> {
                  double idleVoltage =
                      (getPieceType() == GripperConstants.Piece.ALGAE)
                          ? ALGAE_IDLE_VOLTAGE
                          : DEFAULT_IDLE_VOLTAGE;

                  gripperIO.setVoltage(idleVoltage);
                },
                this)
            .withName("idle");

    setDefaultCommand(dynamicIdleCommand);
  }
  // Command idleCommand = setVoltage(-0.5);
  // Command stopCommand = setVoltage(0);

  // Command conditionalDefault = Commands.either(idleCommand, stopCommand, this::hasPiece);

  // setDefaultCommand(conditionalDefault);

  @Override
  public void periodic() {
    gripperIO.updateInputs(gripperInputs);
    Logger.processInputs("RealOutputs/Gripper", gripperInputs);

    // // --- Lights Logic for Main Strip ---
    // if (hasPiece()) {
    //   if (getPieceType() == GripperConstants.Piece.CORAL) { // Assuming Piece.CORAL for white
    // piece
    //     // Flashing white light (e.g., 0.5 seconds per blink cycle)
    //     LEDSegment.MainStrip.setBlinkAnimation(pink, 1.5);
    //     // LEDSegment.MainStrip.setSolidColor(LightsSubsystem.white);
    //     // lights.setControl(new StrobeAnimation(0, 400).withColor(orange));
    //     // lights.setBlinkAnimation();

    //   } else if (getPieceType() == GripperConstants.Piece.ALGAE) {
    //     // Solid green light (green is likely defined in Lights.java)
    //     LEDSegment.MainStrip.setBlinkAnimation(green, 1.5);
    //     // LEDSegment.MainStrip.setSolidColor(LightsSubsystem.green);
    //     // candle.setControl(new StrobeAnimation(0, 400).withColor(orange));

    //   }
    // } else {
    // If no piece is held, let the Lights subsystem default command run,
    // or set a specific "no piece" animation.
    // LEDSegment.MainStrip.clearAnimation();
    // The Lights default command (e.g., setSolidColor(orange)) will take over
    // }
  }

  // public Command lightsPieceIndicator(GripperConstants.Piece piece, double duration) {
  //   Color color =
  //       (piece == GripperConstants.Piece.ALGAE) ? LightsSubsystem.green : LightsSubsystem.white;

  //   return Commands.sequence(
  //           Commands.runOnce(
  //               () -> {
  //                 LEDSegment.MainStrip.setBlinkAnimation(color, 3.0);
  //               },
  //               this.lights),
  //           Commands.waitSeconds(duration))
  //       .finallyDo(
  //           (interrupted) -> {
  //             LEDSegment.MainStrip.clearAnimation();
  //           });
  // }

  public Command placePiece() {
    return Commands.race(
        setVoltage(GripperConstants.gripperPlacementVoltage), Commands.waitSeconds(0.5));
  }

  public Command placePieceAuto() {
    return Commands.race(
        setVoltage(GripperConstants.gripperPlacementVoltage), Commands.waitSeconds(0.25));
  }

  public Command placePieceL1() {
    return Commands.race(
        setVoltage(GripperConstants.gripperPlacementVoltageL1), Commands.waitSeconds(0.5));
    // return setVoltage(GripperConstants.gripperPlacementVoltageL1).until(HAS_PIECE.negate());
  }

  public Command placePieceAlgae() {
    return setVoltage(7); //
    // return Commands.race(setVoltage(7), Commands.waitSeconds(0.5));
    // return setVoltage(GripperConstants.gripperPlacementVoltageL1).until(HAS_PIECE.negate());
  }

  public Command intakeUntilPieceDetected() {

    return setVoltage(GripperConstants.intakeVoltage).until(() -> hasPiece());
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
