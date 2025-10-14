// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lights;

import com.ctre.phoenix6.configs.CANdleConfiguration;
// Base class for all controls
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LedConstants;
import java.util.function.BooleanSupplier;

public class LightsSubsystem extends SubsystemBase {

  BooleanSupplier hasPiece;

  public static final class LightsConstants {
    public static final int CANDLE_PORT = 18;
  }

  private static final CANdle candle;
  private static final boolean isReal = true;

  static {
    if (RobotBase.isReal() && isReal) {
      candle = new CANdle(LightsConstants.CANDLE_PORT, "CANivore");
    } else {
      candle = null;
    }
  }

  public LightsSubsystem(BooleanSupplier hasPieceSupplier) {
    this.hasPiece = hasPieceSupplier;
    if (candle != null) {
      CANdleConfiguration cfg = new CANdleConfiguration();

      /* clear all previous animations */
      for (int i = 0; i < 8; ++i) {
        candle.setControl(new EmptyAnimation(i));
      }

      cfg.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
      cfg.LED.StripType = StripTypeValue.GRB;
      cfg.LED.BrightnessScalar = 0.99;
      candle.getConfigurator().apply(cfg);
    }

    setDefaultCommand(defaultCommand());
  }

  public static void setBrightness(double brightness) {
    if (candle != null) {
      var cfg = new CANdleConfiguration();
      cfg.LED.BrightnessScalar = brightness;
      candle.getConfigurator().apply(cfg.LED);
    }
  }

  public Command defaultCommand() {
    return run(() -> {
          if (DriverStation.isEnabled()) {

            if (hasPiece.getAsBoolean()) {
              candle.setControl(new StrobeAnimation(0, 500).withColor(LedConstants.kOrange));
            } else {
              candle.setControl(new FireAnimation(0, 500));
            }
          } else {
            candle.setControl(new SolidColor(0, 500).withColor(LedConstants.kOrange));
            // candle.setControl(new ColorFlowAnimation(0, 500).withColor(LedConstants.kOrange).);
            // LEDSegment.BatteryIndicator.setSolidColor(LightsSubsystem.red);
          }
        })
        .ignoringDisable(true);
  }

  public Command setColor(RGBWColor color) {
    return Commands.run(() -> candle.setControl(new SolidColor(0, 500).withColor(color)), this);
  }

  @Override
  public void periodic() {}
}
