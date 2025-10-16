// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lights;

import com.ctre.phoenix6.configs.CANdleConfiguration;
// Base class for all controls
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightsSubsystem extends SubsystemBase {
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

  public static final Color orange = new Color(255, 25, 0);
  public static final Color stripOrange = new Color(255, 255, 0);
  public static final Color black = new Color(0, 0, 0);

  // Game piece colors
  public static final Color yellow = new Color(242, 60, 0);
  public static final Color purple = new Color(200, 0, 200);

  // Indicator colors
  public static final Color white = new Color(255, 255, 255);
  public static final Color green = new Color(56, 209, 0);
  public static final Color blue = new Color(8, 32, 255);
  public static final Color red = new Color(255, 0, 0);

  public LightsSubsystem() {
    if (candle != null) {
      CANdleConfiguration cfg = new CANdleConfiguration();
      cfg.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
      cfg.LED.StripType = StripTypeValue.GRB;
      cfg.LED.BrightnessScalar = 1.0;
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
          if (RobotController.getBatteryVoltage() > 12.3) {
            LEDSegment.BatteryIndicator.setSolidColor(LightsSubsystem.green);
          } else {
            LEDSegment.BatteryIndicator.setFadeAnimation(LightsSubsystem.green, 1.0);
            // LEDSegment.BatteryIndicator.setSolidColor(LightsSubsystem.red);
          }

          LEDSegment.MainStrip.clearAnimation();

          if (DriverStation.isEnabled()) {
            // setBrightness(1.0);
            LEDSegment.MainStrip.setSolidColor(purple);
            // LEDSegment.MainStrip.setFireAnimation(.5, .5);
          } else {
            // setBrightness(.5);
            // LEDSegment.MainStrip.setFadeAnimation(stripOrange, 3);
            LEDSegment.MainStrip.setSolidColor(orange);
          }
        })
        .ignoringDisable(true);
  }

  private static RGBWColor toRGBWColor(Color color) {
    return new RGBWColor((int) color.red, (int) color.green, (int) color.blue).scaleBrightness(1);
  }

  public static enum LEDSegment {
    MainStrip(4, 500, 0),
    BatteryIndicator(1, 3, 0);

    public final int startIndex;
    public final int segmentSize;
    public final int animationSlot;

    private LEDSegment(int startIndex, int segmentSize, int animationSlot) {
      this.startIndex = startIndex;
      this.segmentSize = segmentSize;
      this.animationSlot = animationSlot;
    }

    public void setSolidColor(Color color) {
      if (candle != null) {
        clearAnimation();

        RGBWColor rgbw = toRGBWColor(color);
        candle.setControl(new SolidColor(startIndex, segmentSize).withColor(rgbw));
      }
    }

    // private void setAnimationControl(ControlRequest animation) {
    //   if (candle != null) {
    //     candle.setControl(animation);
    //   }
    // }

    public void fullClear() {
      if (candle != null) {
        clearAnimation();
        disableLEDs();
      }
    }

    public void clearAnimation() {
      if (candle != null) {
        candle.setControl(new EmptyAnimation(animationSlot));
      }
    }

    public void disableLEDs() {
      setSolidColor(black);
    }

    // In LEDSegment Enum
    public void setFadeAnimation(Color color, double periodSeconds) {
      RGBWColor rgbw = toRGBWColor(color);

      SingleFadeAnimation fade =
          new SingleFadeAnimation(startIndex, segmentSize)
              .withColor(rgbw)
              .withFrameRate(1.0 / periodSeconds)
              .withSlot(animationSlot);

      candle.setControl(fade);
    }

    /**
     * @param speed Animation speed (0-1)
     * @param cooling Cooling factor (0-1)
     * @param sparking Sparking factor (0-1)
     */
    public void setFireAnimation(double cooling, double sparking) {
      FireAnimation fire =
          new FireAnimation(startIndex, segmentSize)
              .withDirection(AnimationDirectionValue.Forward)
              .withCooling(cooling) // Animation speed is tuned by these
              .withSparking(sparking)
              // .withFrameRate(40.0)
              // No .withSpeed() method
              .withSlot(animationSlot);

      candle.setControl(fire);
    }

    public void setBlinkAnimation(Color color, double periodSeconds) {
      RGBWColor rgbw = toRGBWColor(color);

      double frameRateHz = 1.0 / periodSeconds;

      SingleFadeAnimation blink =
          new SingleFadeAnimation(startIndex, segmentSize)
              .withColor(rgbw)
              .withSlot(animationSlot)
              .withFrameRate(frameRateHz);

      if (candle != null) {
        candle.setControl(blink);
      }
    }
  }

  public static void disableLEDs() {
    setBrightness(0.0);
  }

  public static void enableLEDs() {
    setBrightness(1.0);
  }
}