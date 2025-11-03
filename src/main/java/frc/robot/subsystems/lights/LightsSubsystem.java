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
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
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
      RobotStatusTimer = new Timer();
    } else {
      candle = null;
    }
  }

  static Timer RobotStatusTimer;

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

  enum driveMode {
    disabled,
    teleop,
    autonomous,
    test,
    estop
  }

  driveMode lastDriveMode;

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

          if (DriverStation.isDisabled()) {
            if (lastDriveMode != driveMode.disabled) {
              lastDriveMode = driveMode.disabled;
              RobotStatusTimer.reset();
              RobotStatusTimer.start();
            }
            if (RobotStatusTimer.get() > 60 * 5) {
              animationTrigger.off();
              return;
            }
            switch (((int) RobotStatusTimer.get() / 10) % 3) {
              case 0:
                animationTrigger.flow(red, 30);
                break;
              case 1:
                animationTrigger.solid(white);
                break;
            }
            return;
          }
          if (DriverStation.isTeleop()) {
            if (lastDriveMode != driveMode.teleop) {
              lastDriveMode = driveMode.teleop;
              RobotStatusTimer.reset();
              RobotStatusTimer.start();
            }
            return;
          }
          if (DriverStation.isAutonomous()) {
            if (lastDriveMode != driveMode.autonomous) {
              lastDriveMode = driveMode.autonomous;
              RobotStatusTimer.reset();
              RobotStatusTimer.start();
            }
            return;
          }
          if (DriverStation.isTest()) {
            if (lastDriveMode != driveMode.test) {
              lastDriveMode = driveMode.test;
              RobotStatusTimer.reset();
              RobotStatusTimer.start();
            }
            return;
          }
          if (DriverStation.isEStopped()) {
            if (lastDriveMode != driveMode.estop) {
              lastDriveMode = driveMode.estop;
              RobotStatusTimer.reset();
              RobotStatusTimer.start();
            }
            animationTrigger.strobe(red, 0.1);
            return;
          }
        })
        .ignoringDisable(true);
  }

  // private static RGBWColor toRGBWColor(Color color) {
  //   return new RGBWColor((int) color.red, (int) color.green, (int)
  // color.blue).scaleBrightness(1);
  // }

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
     */

    public void setSolidColor(RGBWColor color) {
      if (candle == null) return;

      SolidColor solid = new SolidColor(startIndex, startIndex + segmentSize - 1).withColor(color);

      candle.setControl(solid);
    }

    public void setStrobeAnimationPeriod(RGBWColor color, double periodSeconds) {
      if (candle == null) return;

      RGBWColor rgbw = color;

      double frameRateHz = 40.0 / periodSeconds; // Hz of 40 = 1 cycle per second

      StrobeAnimation strobe =
          new StrobeAnimation(startIndex, startIndex + segmentSize - 1)
              .withColor(rgbw)
              .withSlot(animationSlot)
              .withFrameRate(frameRateHz);

      candle.setControl(strobe);
    }

    public void setStrobeAnimationHertz(RGBWColor color, double hz) {
      if (candle == null) return;

      RGBWColor rgbw = color;

      StrobeAnimation strobe =
          new StrobeAnimation(startIndex, startIndex + segmentSize - 1)
              .withColor(rgbw)
              .withSlot(animationSlot)
              .withFrameRate(hz);

      candle.setControl(strobe);
    }

    public void setFadeAnimationPeriod(RGBWColor color, double periodSeconds) {
      if (candle == null) return;

      SingleFadeAnimation fade =
          new SingleFadeAnimation(startIndex, startIndex + segmentSize - 1)
              .withColor(color)
              .withFrameRate(1.0 / periodSeconds)
              .withSlot(animationSlot);

      candle.setControl(fade);
    }

    public void setFadeAnimationFPS(RGBWColor color, double fps) {
      if (candle == null) return;

      SingleFadeAnimation fade =
          new SingleFadeAnimation(startIndex, startIndex + segmentSize - 1)
              .withColor(color)
              .withFrameRate(fps)
              .withSlot(animationSlot);

      candle.setControl(fade);
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
      LEDSegment.MainStrip.setStrobeAnimationPeriod(color, period);
      LEDSegment.MainStripFront.clearAnimation();
      LEDSegment.MainStripBack.clearAnimation();
    }

    public static void fade(RGBWColor color, double period) {
      LEDSegment.MainStrip.setFadeAnimationPeriod(color, period);
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
