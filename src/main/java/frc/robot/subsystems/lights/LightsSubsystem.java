// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lights;

import com.ctre.phoenix6.configs.CANdleConfiguration;
// Base class for all controls
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.controls.TwinkleOffAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;

public class LightsSubsystem extends SubsystemBase {
  public static final class LightsConstants {
    public static final int CANDLE_PORT = 18;
  }

  private static final CANdle candle;
  private static final boolean isReal = true;

  static {
    if (RobotBase.isReal() && isReal) {
      candle = new CANdle(LightsConstants.CANDLE_PORT, "CANivore");
      RobotStatusTimer = new Timer();
    } else {
      candle = null;
    }
  }

  public static BooleanSupplier isIntakingSup =
      (() -> {
        return false;
      });
  public static BooleanSupplier isStraightSup =
      (() -> {
        return false;
      });
  public static BooleanSupplier isCradledSup =
      (() -> {
        return false;
      });
  public static BooleanSupplier isLoadedSup =
      (() -> {
        return false;
      });

  static Timer RobotStatusTimer;
  static boolean reachedEndOfMatch = false;

  public static final RGBWColor orange = new RGBWColor(255, 25, 0);
  public static final RGBWColor black = new RGBWColor(0, 0, 0);

  // Game piece colors
  public static final RGBWColor yellow = new RGBWColor(242, 60, 0);
  public static final Color purple = new Color(200, 0, 200);

  // Indicator colors
  public static final RGBWColor white = new RGBWColor(255, 255, 255);
  public static final RGBWColor green = new RGBWColor(56, 209, 0);
  public static final RGBWColor blue = new RGBWColor(8, 32, 255);
  public static final RGBWColor red = new RGBWColor(255, 0, 0);

  // public static final RGBWColor orange = new RGBWColor(230, 25, 0);

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

  enum DriveMode {
    disabled,
    teleop,
    autonomous,
    test,
    estop
  }

  DriveMode lastDriveMode;

  public Command defaultCommand() {
    return run(() -> {
          // if (RobotController.getBatteryVoltage() > 12.3) {
          //   // LEDSegment.BatteryIndicator.setSolidColor(LightsSubsystem.green);
          // } else {
          //   // LEDSegment.BatteryIndicator.setFadeAnimation(LightsSubsystem.green, 1.0);
          //   // LEDSegment.BatteryIndicator.setSolidColor(LightsSubsystem.red);
          // }

          // LEDSegment.MainStrip.clearAnimation();

          // if (DriverStation.isEnabled()) {
          //   //candle.setControl(new FireAnimation(0, 400));
          // } else {
          //   if (DriverStation.isDisabled())
          //   // candle.setControl(new SolidColor(0, 400).withColor(orange));
          //   //candle.setControl(new SingleFadeAnimation(0, 400).withColor(orange));
          //   // setBrightness(.5);
          //   // LEDSegment.MainStrip.setFadeAnimation(stripOrange, 3);
          //   // LEDSegment.MainStrip.setSolidColor(orange);
          // }

          // Brown out is when voltage is (v < 8)

          if (!DriverStation.isDSAttached()) {
            animationTrigger.fade(white, 5);
            return;
          }
          if (DriverStation.isEStopped()) {
            if (lastDriveMode != DriveMode.estop) {
              lastDriveMode = DriveMode.estop;
              RobotStatusTimer.reset();
              // RobotStatusTimer.start();
            }
            animationTrigger.strobe(red, 0.1);
            return;
          }
          if (DriverStation.isDisabled()) {
            if (lastDriveMode != DriveMode.disabled) {
              lastDriveMode = DriveMode.disabled;
              RobotStatusTimer.reset();
              // RobotStatusTimer.start();
            }
            if (RobotStatusTimer.get() > 300) { // Turn off after 5 minutes
              animationTrigger.off();
              return;
            }

            // if (reachedEndOfMatch) {
            //   animationTrigger.rainbow(1, false);
            //   return;
            // }

            // switch (((int) RobotStatusTimer.get() / 10) % 2) {
            //   case 0:
            //     animationTrigger.flow(red, 30, false);
            //     break;
            //   case 1:
            //     animationTrigger.solid(white);
            //     break;
            // }
            animationTrigger.fireOverdrive();
            return;
          }

          if (DriverStation.isTeleop()) {
            if (lastDriveMode != DriveMode.teleop) {
              lastDriveMode = DriveMode.teleop;
              RobotStatusTimer.reset();
              // RobotStatusTimer.start();
            }

            if (isLoadedSup.getAsBoolean()) {
              animationTrigger.fade(white, 0.25);
              return;
            }
            if (isCradledSup.getAsBoolean()) {
              animationTrigger.strobe(blue, 0.25);
              return;
            }
            if (isStraightSup.getAsBoolean()) {
              animationTrigger.fade(yellow, 0.25);
              return;
            }
            if (isIntakingSup.getAsBoolean()) {
              animationTrigger.fade(orange, 0.25);
              return;
            }

            double matchTimer = RobotStatusTimer.get();
            if (matchTimer < 135) {
              animationTrigger.fire();
              return;
            } else if (matchTimer < 140) {
              animationTrigger.fade(green, 0.5);
              return;
            } else if (matchTimer < 145) {
              animationTrigger.fade(yellow, 0.25);
              return;
            } else if (matchTimer < 150) {
              animationTrigger.strobe(red, 0.2);
              return;
            }

            animationTrigger.fade(orange, 2);
            return;
          }
          if (DriverStation.isAutonomous()) {
            if (lastDriveMode != DriveMode.autonomous) {
              lastDriveMode = DriveMode.autonomous;
              RobotStatusTimer.reset();
              // RobotStatusTimer.start();
            }
            return;
          }
          if (DriverStation.isTest()) {
            if (lastDriveMode != DriveMode.test) {
              lastDriveMode = DriveMode.test;
              RobotStatusTimer.reset();
              // RobotStatusTimer.start();
            }
            return;
          }
        })
        .ignoringDisable(true);
  }

  public static enum LEDSegment {
    BatteryIndicator(1, 3, 0),
    MainStrip(8, 115, 0),
    MainStripFront(8, 62, 1, false),
    MainStripBack(70, 53, 2, true);

    // Parameters
    public final int startIndex;
    public final int segmentSize;
    public final int animationSlot;
    public final boolean reversed;

    // Constructors
    private LEDSegment(int startIndex, int segmentSize, int animationSlot) {
      this.startIndex = startIndex;
      this.segmentSize = segmentSize;
      this.animationSlot = animationSlot;
      this.reversed = false;
    }

    private LEDSegment(int startIndex, int segmentSize, int animationSlot, boolean reversed) {
      this.startIndex = startIndex;
      this.segmentSize = segmentSize;
      this.animationSlot = animationSlot;
      this.reversed = reversed;
    }

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
      // setSolidColor(black);
    }

    /*
     * A note on animation speeds
     *
     * Speeds run on hertz, meaning the withFrameRate is how many times it updates per second
     * The minimum speed is 20 hz, meaning 20 fps
     * The maximum speed is 1000 hz, meaning 1000 fps
     *
     * Each animation has a different hz per cycle ratio
     */

    public void setSolidColor(RGBWColor color) {
      if (candle == null) return;

      SolidColor solid = new SolidColor(startIndex, startIndex + segmentSize - 1).withColor(color);

      candle.setControl(solid);
    }

    public void setStrobeAnimation(RGBWColor color, double periodSeconds) {
      if (candle == null) return;

      double frameRateHz = 40.0 / periodSeconds; // Hz of 40 = 1 cycle per second

      StrobeAnimation strobe =
          new StrobeAnimation(startIndex, startIndex + segmentSize - 1)
              .withColor(color)
              .withSlot(animationSlot)
              .withFrameRate(frameRateHz);

      candle.setControl(strobe);
    }

    public void setFadeAnimation(RGBWColor color, double periodSeconds) {
      if (candle == null) return;

      double frameRateHz = 200.0 / periodSeconds; // Hz of ????? = 1 cycle per second

      SingleFadeAnimation fade =
          new SingleFadeAnimation(startIndex, startIndex + segmentSize - 1)
              .withColor(color)
              .withFrameRate(frameRateHz)
              .withSlot(animationSlot);

      candle.setControl(fade);
    }

    public void setRGBFadeAnimation(double periodSeconds) {
      if (candle == null) return;

      double frameRateHz = 200.0 / periodSeconds; // Hz of ????? = 1 cycle per second

      RgbFadeAnimation fade =
          new RgbFadeAnimation(startIndex, startIndex + segmentSize - 1)
              .withFrameRate(frameRateHz)
              .withSlot(animationSlot);

      candle.setControl(fade);
    }

    public void setRainbowAnimationPeriod(double periodSeconds, boolean inverted) {
      if (candle == null) return;

      double frameRateHz = 120.0 / periodSeconds; // Hz of ????? = 1 cycle per second

      RainbowAnimation rainbow =
          new RainbowAnimation(startIndex, startIndex + segmentSize - 1)
              .withFrameRate(frameRateHz)
              .withDirection(
                  (reversed) ? AnimationDirectionValue.Backward : AnimationDirectionValue.Forward)
              .withSlot(animationSlot);

      candle.setControl(rainbow);
    }

    public void setFlowAnimation(RGBWColor color, double periodSeconds, boolean inverted) {
      if (candle == null) return;

      double frameRateHz = 2.0 * segmentSize / periodSeconds; // Hz of ????? = 1 cycle per second

      ColorFlowAnimation flow =
          new ColorFlowAnimation(startIndex, startIndex + segmentSize - 1)
              .withColor(color)
              .withFrameRate(frameRateHz)
              .withDirection(
                  (reversed != inverted)
                      ? AnimationDirectionValue.Backward
                      : AnimationDirectionValue.Forward)
              .withSlot(animationSlot);

      candle.setControl(flow);
    }

    public void setLarsonAnimation(RGBWColor color, double periodSeconds, int size) {
      if (candle == null) return;

      double frameRateHz =
          2.0 * (segmentSize - size) / periodSeconds; // Hz of ????? = 1 cycle per second

      LarsonAnimation larson =
          new LarsonAnimation(startIndex, startIndex + segmentSize - 1)
              .withColor(color)
              .withFrameRate(frameRateHz)
              .withSize(animationSlot)
              .withBounceMode(LarsonBounceValue.Front)
              .withSlot(animationSlot);

      candle.setControl(larson);
    }

    public void setTwinkleAnimation(RGBWColor color, double periodSeconds, double percentage) {
      if (candle == null) return;

      double frameRateHz = 2.0 * segmentSize / periodSeconds; // Hz of ????? = 1 cycle per second
      double amount = segmentSize * percentage;

      TwinkleAnimation twinkle =
          new TwinkleAnimation(startIndex, startIndex + segmentSize - 1)
              .withColor(color)
              .withFrameRate(frameRateHz)
              .withMaxLEDsOnProportion(amount)
              .withSlot(animationSlot);

      candle.setControl(twinkle);
    }

    public void setTwinkleOffAnimation(RGBWColor color, double periodSeconds, double percentage) {
      if (candle == null) return;

      double frameRateHz = 2.0 * segmentSize / periodSeconds; // Hz of ????? = 1 cycle per second
      double amount = segmentSize * percentage;

      TwinkleOffAnimation twinkle =
          new TwinkleOffAnimation(startIndex, startIndex + segmentSize - 1)
              .withColor(color)
              .withFrameRate(frameRateHz)
              .withMaxLEDsOnProportion(amount)
              .withSlot(animationSlot);

      candle.setControl(twinkle);
    }

    /**
     * @param speed Animation speed (0-1)
     * @param cooling Cooling factor (0-1)
     * @param sparking Sparking factor (0-1)
     * @param inverted Invert setting (false/true) {false = bottom-up}
     * @param fps Frames per second (2-1000)
     */
    public void setFireAnimation(double cooling, double sparking, boolean inverted, double fps) {
      if (candle == null) return;

      FireAnimation fire =
          new FireAnimation(startIndex, startIndex + segmentSize - 1)
              .withCooling(cooling)
              .withSparking(sparking)
              .withFrameRate(fps)
              .withDirection(
                  (reversed) ? AnimationDirectionValue.Backward : AnimationDirectionValue.Forward)
              .withSlot(animationSlot);

      candle.setControl(fire);
    }
  }

  public static void disableLEDs() {
    setBrightness(0.0);
  }

  public static void enableLEDs() {
    setBrightness(1.0);
  }

  static class animationTrigger {
    public static void off() {
      LEDSegment.MainStrip.clearAnimation();
      LEDSegment.MainStripFront.clearAnimation();
      LEDSegment.MainStripBack.clearAnimation();
    }

    public static void solid(RGBWColor color) {
      LEDSegment.MainStrip.setSolidColor(color);
      LEDSegment.MainStripFront.clearAnimation();
      LEDSegment.MainStripBack.clearAnimation();
    }

    public static void strobe(RGBWColor color, double period) {
      LEDSegment.MainStrip.setStrobeAnimation(color, period);
      LEDSegment.MainStripFront.clearAnimation();
      LEDSegment.MainStripBack.clearAnimation();
    }

    public static void fade(RGBWColor color, double period) {
      LEDSegment.MainStrip.setFadeAnimation(color, period);
      LEDSegment.MainStripFront.clearAnimation();
      LEDSegment.MainStripBack.clearAnimation();
    }

    public static void rgbFade(double period) {
      LEDSegment.MainStrip.setRGBFadeAnimation(period);
      LEDSegment.MainStripFront.clearAnimation();
      LEDSegment.MainStripBack.clearAnimation();
    }

    public static void rainbow(double period, boolean inverted) {
      LEDSegment.MainStrip.setRainbowAnimationPeriod(period, inverted);
      LEDSegment.MainStripFront.clearAnimation();
      LEDSegment.MainStripBack.clearAnimation();
    }

    public static void flow(RGBWColor color, double period, boolean inverted) {
      LEDSegment.MainStrip.setFlowAnimation(color, period, inverted);
      LEDSegment.MainStripFront.clearAnimation();
      LEDSegment.MainStripBack.clearAnimation();
    }

    public static void larson(RGBWColor color, double period, int size) {
      LEDSegment.MainStrip.setLarsonAnimation(color, period, size);
      LEDSegment.MainStripFront.clearAnimation();
      LEDSegment.MainStripBack.clearAnimation();
    }

    public static void twinkle(RGBWColor color, double period, int percentage) {
      LEDSegment.MainStrip.setTwinkleAnimation(color, period, percentage);
      LEDSegment.MainStripFront.clearAnimation();
      LEDSegment.MainStripBack.clearAnimation();
    }

    public static void twinkleOff(RGBWColor color, double period, int percentage) {
      LEDSegment.MainStrip.setTwinkleOffAnimation(color, period, percentage);
      LEDSegment.MainStripFront.clearAnimation();
      LEDSegment.MainStripBack.clearAnimation();
    }

    public static void fire() {
      LEDSegment.MainStrip.clearAnimation();
      LEDSegment.MainStripFront.setFireAnimation(0.4, 0.4, false, 40);
      LEDSegment.MainStripBack.setFireAnimation(0.4, 0.4, false, 40);
    }

    public static void fireOverdrive() {
      LEDSegment.MainStrip.clearAnimation();
      LEDSegment.MainStripFront.setFireAnimation(0.35, 0.5, false, 55);
      LEDSegment.MainStripBack.setFireAnimation(0.35, 0.5, false, 55);
    }
  }
}
