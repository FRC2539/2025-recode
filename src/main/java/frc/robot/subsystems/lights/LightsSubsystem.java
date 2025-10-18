// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lights;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
// Base class for all controls
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
// import com.ctre.phoenix6.controls.TwinkleAnimation;
// import com.ctre.phoenix6.controls.TwinkleOffAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightsSubsystem extends SubsystemBase {
  public static final class LightsConstants {
    public static final int CANDLE_PORT = 18;

    public static final int SENSOR_PORT = 0;
  }

  private BooleanSupplier algaeMode = (() -> {return false;});

  private static final CANdle candle;

  private static final boolean isReal = true;

  // TO ENABLE/DISABLE FEATURES
  static LoggedNetworkBoolean timerEnabled = new LoggedNetworkBoolean("LIGHTS TIMER", true);
  static LoggedNetworkBoolean progressEnabled = new LoggedNetworkBoolean("LIGHTS PROGRESS BARS", true);
  static LoggedNetworkBoolean alignProximityEnabled = new LoggedNetworkBoolean("LIGHTS AIGNING PROXIMITY", true);
  static LoggedNetworkBoolean emotesEnabled = new LoggedNetworkBoolean("LIGHTS EMOTES", true);

  static {
    if (RobotBase.isReal() && isReal) {
      candle = new CANdle(LightsConstants.CANDLE_PORT, "roboRIO");
    } else {
      candle = null;
    }
    // Establish the timer for later use
    LightsControlModule.RobotStatusTimer = new Timer();
  }

  // #region Color Defs

  // Team colors
  public static final RGBWColor orange = new RGBWColor(255, 25, 0);
  public static final RGBWColor black = new RGBWColor(0, 0, 0);

  // Game piece colors
  public static final RGBWColor yellow = new RGBWColor(242, 60, 0);
  public static final RGBWColor purple = new RGBWColor(184, 0, 185);

  // Indicator colors
  public static final RGBWColor white = new RGBWColor(255, 230, 220);
  public static final RGBWColor green = new RGBWColor(56, 209, 0);
  public static final RGBWColor blue = new RGBWColor(8, 32, 255);
  public static final RGBWColor red = new RGBWColor(255, 0, 0);
  public static final RGBWColor gray = new RGBWColor(75, 75, 75);
  public static final RGBWColor brown = new RGBWColor(170, 130, 50);

  // #endregion
  
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
          // LEDSegment.BatteryIndicator.fullClear();
          // LEDSegment.DriverstationIndicator.fullClear();
          // LEDSegment.ExtraAIndicator.fullClear();
          // LEDSegment.ExtraBIndicator.fullClear();
          // LEDSegment.PivotEncoderIndicator.fullClear();
          LightsControlModule.update();
        })
        .ignoringDisable(true);
  }

  public void setAlgaeModeSupplier(BooleanSupplier algaeMode) {
    this.algaeMode = algaeMode;
  }

  public Command clearSegmentCommand(LEDSegment segment) {
    return runOnce(
        () -> {
          segment.clearAnimation();
          segment.disableLEDs();
        });
  }

  // Condensed storage for animations, called when setting animations
  public static class LightsControlModule {
    public enum RobotStatus {
      Disabled,
      Teleop,
      Autonomous,
      Test
    }

    enum mode {
      disabled,
      paused,
      manual,
      strobe,
      fade,
      waiting,
      flow,
      disabledLoaded,
      autoLoaded,
      fire,
      autoFire,
      rainbow,
      intake,
      brownOut,
      alignLeft,
      alignLeftNear,
      alignLeftFar,
      alignRight,
      alignRightNear,
      alignRightFar,
      alignCenter,
      alignCenterNear,
      alignCenterFar,
      timeRemainingA,
      timeRemainingB,
      timeRemainingC,
      testProgress,
      matchProgress,
      twinkle,
      twinkleOff
    }

    static mode lightMode = mode.disabled;

    static double alignToleranceMin = 5;
    static double alignToleranceMax = 100;

    // #region Robot Status
    static Timer RobotStatusTimer;
    static RobotStatus robotStatus = RobotStatus.Disabled;

    public static void setRobotStatus(RobotStatus newStatus) {
      robotStatus = newStatus;
      // Restart the timer for the new status
      if (timerEnabled.get()) {
        RobotStatusTimer.reset();
        RobotStatusTimer.start();
      }
    }
    // #endregion
    // #region Suppliers
    static BooleanSupplier hasPiece = (() -> {return false;});
    static BooleanSupplier isAligning = (() -> {return false;});
    static IntSupplier alignMode = (() -> {return 0;});
    static DoubleSupplier batteryVoltage = (() -> {return 0;});
    static DoubleSupplier opControllerLeftX = (() -> {return 0;});
    static DoubleSupplier opControllerLeftY = (() -> {return 0;});
    static DoubleSupplier opControllerRightX = (() -> {return 0;});
    static DoubleSupplier opControllerRightY = (() -> {return 0;});
    static DoubleSupplier opControllerLeftMagnitude =
      (() -> {return Math.hypot(opControllerLeftX.getAsDouble(), opControllerLeftY.getAsDouble());});
    static DoubleSupplier opControllerRightMagnitude =
      (() -> {return Math.hypot(opControllerRightX.getAsDouble(), opControllerRightY.getAsDouble());});

    public static void Supplier_hasPiece(BooleanSupplier sup) {
      hasPiece = sup;
    }

    public static void Supplier_isAligning(BooleanSupplier sup) {
      isAligning = sup;
    }

    public static void Supplier_alignMode(IntSupplier sup) {
      alignMode = sup;
    }

    public static void Supplier_batteryVoltage(DoubleSupplier sup) {
      batteryVoltage = sup;
    }

    public static void Supplier_opControllerLeftX(DoubleSupplier sup) {
      opControllerLeftX = sup;
    }

    public static void Supplier_opControllerLeftY(DoubleSupplier sup) {
      opControllerLeftY = sup;
    }

    public static void Supplier_opControllerRightX(DoubleSupplier sup) {
      opControllerRightX = sup;
    }

    public static void Supplier_opControllerRightY(DoubleSupplier sup) {
      opControllerRightY = sup;
    }
    public static double DistanceToTag = 0;
    // #endregion

    enum cardinalDirection {
      North,
      East,
      South,
      West
    }

    static cardinalDirection getJoystickCardinal(double x, double y) {
      if (y >= -x) {
        if (y >= x) return cardinalDirection.North;
        return cardinalDirection.East;
      } else {
        if (y >= x) return cardinalDirection.West;
        return cardinalDirection.South;
      }
    }

    public static void update() {
      // #region Emote Wheel
      if (opControllerLeftMagnitude.getAsDouble() > 0.05) {
        cardinalDirection dir =
            getJoystickCardinal(opControllerLeftX.getAsDouble(), opControllerLeftY.getAsDouble());
        switch (dir) {
          case North:
            rainbow();
            break;
          case East:
            testProgress();
            break;
          case South:
            intake();
            break;
          case West:
            disabledLoaded();
            break;
        }
        return;
      }
      if (opControllerRightMagnitude.getAsDouble() > 0.05) {
        cardinalDirection dir =
            getJoystickCardinal(opControllerRightX.getAsDouble(), opControllerRightY.getAsDouble());
        switch (dir) {
          case North:
            matchProgress();
            break;
          case East:
            // twinkle();
            break;
          case South:
            break;
          case West:
            // twinkleOff();
            break;
        }
        return;
      }
      // #endregion
      // #region Disabled Logic
      if (robotStatus == RobotStatus.Disabled) {
        if (batteryVoltage.getAsDouble() <= 11) {
          brownOut();
          return;
        }
        if (timerEnabled.get()) {
          double seconds = RobotStatusTimer.get();
          if (seconds > 300) {
            rainbow();
            return;
          }
        }
        // Has Piece
        if (hasPiece.getAsBoolean()) {
          fire();
          return;
        }
        flow();
        return;
      }
      // #endregion
      // #region Enabled Logic
      if (robotStatus == RobotStatus.Teleop) {
        // Align Mode
        if (isAligning.getAsBoolean()) {
          int alignModeInt = alignMode.getAsInt();
          if (alignModeInt == 1) { // Align Left
            alignLeft(DistanceToTag); // TODO: test DistanceToTag
            return;
          }
          if (alignModeInt == 2) { // Align Right
            alignRight(DistanceToTag); // TODO: test DistanceToTag
            return;
          }
          if (alignModeInt == 3) { // Align Center (Algae)
            alignCenter(DistanceToTag); // TODO: test DistanceToTag
            return;
          }
          // Idle
          fade();
          return;
        }
        // HasPiece
        if (hasPiece.getAsBoolean()) {
          strobe();
          return;
        }

        // Brown Out
        if (batteryVoltage.getAsDouble() <= 11) {
          brownOut();
          return;
        }

        // Idle
        if (timerEnabled.get()) {
          double seconds = RobotStatusTimer.get(); // Teleop length = 2:15, which is 135 seconds
          if (!LightsSubsystem.timerEnabled.get() || seconds < 120) {
            fire();
            return;
          } else if (seconds < 125) {
            timeRemainingA();
            return;
          } else if (seconds < 130) {
            timeRemainingB();
            return;
          } else if (seconds < 135) {
            timeRemainingC();
            return;
          } else {
            rainbow();
            return;
          }
        }
        else {
          fire();
          return;
        }
      }
      // #endregion
      // #region Autonomous Logic
      if (robotStatus == RobotStatus.Autonomous) {
        // Brown Out
        if (batteryVoltage.getAsDouble() <= 11) {
          brownOut();
          return;
        }

        // Has Piece
        if (hasPiece.getAsBoolean()) {
          autoLoaded();
          return;
        }

        // Idle
        if (timerEnabled.get()) {
          double seconds = RobotStatusTimer.get(); // Teleop length = 30 seconds
          if (!LightsSubsystem.timerEnabled.get() || seconds < 15) {
            autoFire();
            return;
          } else if (seconds < 20) {
            timeRemainingA();
            return;
          } else if (seconds < 25) {
            timeRemainingB();
            return;
          } else if (seconds < 30) {
            timeRemainingC();
            return;
          } else {
            waiting();
            return;
          }
        }
        else {
          autoFire();
          return;
        }
      }
      // #endregion
      autoFire();
    }

    // #region Mode Methods
    public static void clearAnimation() {
      if (lightMode == mode.paused) return;
      lightMode = mode.paused;

      LEDSegment.MainStrip.clearAnimation();
      LEDSegment.MainStripBack.clearAnimation();
      LEDSegment.MainStripFront.clearAnimation();
    }

    public static void fullClear() {
      if (lightMode == mode.paused) return;
      lightMode = mode.paused;

      LEDSegment.MainStrip.fullClear();
      LEDSegment.MainStripBack.fullClear();
      LEDSegment.MainStripFront.fullClear();
    }

    public static void strobe() {
      if (lightMode == mode.strobe) return;
      lightMode = mode.strobe;

      LEDSegment.MainStrip.setStrobeAnimation(white, 0.3);
      LEDSegment.MainStripBack.clearAnimation();
      LEDSegment.MainStripFront.clearAnimation();
    }

    public static void fade() {
      if (lightMode == mode.fade) return;
      lightMode = mode.fade;

      LEDSegment.MainStrip.setFadeAnimation(orange, 0.5);
      LEDSegment.MainStripBack.clearAnimation();
      LEDSegment.MainStripFront.clearAnimation();
    }

    public static void waiting() {
      if (lightMode == mode.waiting) return;
      lightMode = mode.waiting;

      LEDSegment.MainStrip.setFadeAnimation(white, 0.75);
      LEDSegment.MainStripBack.clearAnimation();
      LEDSegment.MainStripFront.clearAnimation();
    }

    public static void flow() {
      if (lightMode == mode.flow) return;
      lightMode = mode.flow;

      LEDSegment.MainStrip.setFlowAnimation(orange, 0.75, false);
      LEDSegment.MainStripBack.clearAnimation();
      LEDSegment.MainStripFront.clearAnimation();
    }

    public static void disabledLoaded() {
      if (lightMode == mode.disabledLoaded) return;
      lightMode = mode.disabledLoaded;

      LEDSegment.MainStrip.setFlowAnimation(green, 0.75, false);
      LEDSegment.MainStripBack.clearAnimation();
      LEDSegment.MainStripFront.clearAnimation();
    }

    public static void autoLoaded() {
      if (lightMode == mode.autoLoaded) return;
      lightMode = mode.autoLoaded;

      LEDSegment.MainStrip.setStrobeAnimation(orange, 0.3);
      LEDSegment.MainStripBack.clearAnimation();
      LEDSegment.MainStripFront.clearAnimation();
    }

    public static void fire() {
      if (lightMode == mode.fire) return;
      lightMode = mode.fire;

      LEDSegment.MainStrip.clearAnimation();
      LEDSegment.MainStripBack.setFireAnimation(1, false);
      LEDSegment.MainStripFront.setFireAnimation(1, false);
    }

    public static void autoFire() {
      if (lightMode == mode.autoFire) return;
      lightMode = mode.autoFire;

      LEDSegment.MainStrip.clearAnimation();
      LEDSegment.MainStripBack.setFireOverdriveAnimation(1.75, false);
      LEDSegment.MainStripFront.setFireOverdriveAnimation(1.75, false);
    }

    public static void rainbow() {
      if (lightMode == mode.rainbow) return;
      lightMode = mode.rainbow;

      LEDSegment.MainStrip.setRainbowAnimation(0.8, false);
      LEDSegment.MainStripBack.clearAnimation();
      LEDSegment.MainStripFront.clearAnimation();
    }

    public static void intake() {
      if (lightMode == mode.intake) return;
      lightMode = mode.intake;

      LEDSegment.MainStrip.setStrobeAnimation(yellow, 0.25);
      LEDSegment.MainStripBack.clearAnimation();
      LEDSegment.MainStripFront.clearAnimation();
    }

    public static void brownOut() {
      if (lightMode == mode.brownOut) return;
      lightMode = mode.brownOut;

      LEDSegment.MainStrip.setFadeAnimation(brown, 0.8);
      LEDSegment.MainStripBack.clearAnimation();
      LEDSegment.MainStripFront.clearAnimation();
    }

    public static void alignLeft(double distance) {

      if (distance < alignToleranceMin) {
        if (lightMode != mode.alignLeftNear) {
          LEDSegment.MainStrip.setColor(green);
          LEDSegment.MainStripBack.clearAnimation();
          LEDSegment.MainStripFront.clearAnimation();
          lightMode = mode.alignLeftNear;
        }
      } else if (distance < alignToleranceMax) {
        if (lightMode != mode.alignLeft) {
          LEDSegment.MainStripFront.progressCount = 0;
          LEDSegment.MainStrip.clearAnimation();
          LEDSegment.MainStripBack.setStrobeAnimation(blue, 0.25);
          LEDSegment.MainStripFront.setColor(red);
          lightMode = mode.alignLeft;
        }
        updateProgressBar(LEDSegment.MainStripFront, distance);
      } else {
        if (lightMode != mode.alignLeftFar) {
          LEDSegment.MainStrip.clearAnimation();
          LEDSegment.MainStripBack.setStrobeAnimation(yellow, 0.15);
          LEDSegment.MainStripFront.setColor(red);
          lightMode = mode.alignLeftFar;
        }
      }
    }

    public static void alignRight(double distance) {
      if (distance < alignToleranceMin) {
        if (lightMode != mode.alignRightNear) {
          LEDSegment.MainStrip.setColor(green);
          LEDSegment.MainStripBack.clearAnimation();
          LEDSegment.MainStripFront.clearAnimation();
          lightMode = mode.alignRightNear;
        }
      } else if (distance < alignToleranceMax) {
        if (lightMode != mode.alignRight) {
          LEDSegment.MainStripBack.progressCount = 0;
          LEDSegment.MainStrip.clearAnimation();
          LEDSegment.MainStripBack.setColor(red);
          LEDSegment.MainStripFront.setStrobeAnimation(blue, 0.3);
          lightMode = mode.alignRight;
        }
        updateProgressBar(LEDSegment.MainStripBack, distance);
      } else {
        if (lightMode != mode.alignRightFar) {
          LEDSegment.MainStrip.clearAnimation();
          LEDSegment.MainStripBack.setColor(red);
          LEDSegment.MainStripFront.setStrobeAnimation(yellow, 0.15);
          lightMode = mode.alignRightFar;
        }
      }
    }

    public static void alignCenter(double distance) {
      if (lightMode != mode.alignCenter) LEDSegment.MainStrip.progressCount = 0;

      // The idea here: blink according to the directon we want to drive
      // Functionality is not correctly implemented at this time
      // This might be harder, because we have to calculate a normal for the targetPosition

      if (distance < alignToleranceMin) {
        if (lightMode != mode.alignCenterNear) {
          LEDSegment.MainStrip.setColor(green);
          LEDSegment.MainStripBack.clearAnimation();
          LEDSegment.MainStripFront.clearAnimation();
          lightMode = mode.alignCenterNear;
        }
      } else if (distance < alignToleranceMax) {
        if (lightMode != mode.alignCenter) {
          LEDSegment.MainStripBack.progressCount = 0;
          LEDSegment.MainStripFront.progressCount = 0;
          LEDSegment.MainStrip.clearAnimation();
          LEDSegment.MainStripBack.setColor(yellow);
          LEDSegment.MainStripFront.setColor(yellow);
          lightMode = mode.alignCenter;
        }
        updateProgressBar(LEDSegment.MainStripBack, distance);
        updateProgressBar(LEDSegment.MainStripFront, distance);
      } else {
        if (lightMode != mode.alignCenterFar) {
          LEDSegment.MainStrip.clearAnimation();
          LEDSegment.MainStripBack.setColor(red);
          LEDSegment.MainStripFront.setColor(red);
          lightMode = mode.alignCenterFar;
        }
      }
    }

    public static void timeRemainingA() {
      if (lightMode == mode.timeRemainingA) return;
      lightMode = mode.timeRemainingA;

      LEDSegment.MainStrip.setColor(green);
      LEDSegment.MainStripBack.setFadeAnimation(green, 0.75);
      LEDSegment.MainStripFront.setFadeAnimation(green, 0.75);
    }

    public static void timeRemainingB() {
      if (lightMode == mode.timeRemainingB) return;
      lightMode = mode.timeRemainingB;

      LEDSegment.MainStrip.setColor(yellow);
      LEDSegment.MainStripBack.setFadeAnimation(yellow, 0.85);
      LEDSegment.MainStripFront.setFadeAnimation(yellow, 0.85);
    }

    public static void timeRemainingC() {
      if (lightMode == mode.timeRemainingC) return;
      lightMode = mode.timeRemainingC;

      LEDSegment.MainStrip.setColor(red);
      LEDSegment.MainStripBack.setFadeAnimation(red, 1);
      LEDSegment.MainStripFront.setFadeAnimation(red, 1);
    }

    public static void testProgress() {
      if (lightMode != mode.testProgress) {
        LEDSegment.MainStripBack.progressCount = 0;
        LEDSegment.MainStripFront.progressCount = 0;
        LEDSegment.MainStrip.clearAnimation();
        LEDSegment.MainStripBack.setColor(red);
        LEDSegment.MainStripFront.setColor(red);
        lightMode = mode.testProgress;
      }
      double distance = 150 * opControllerLeftX.getAsDouble();
      updateProgressBar(LEDSegment.MainStripBack, distance);
      updateProgressBar(LEDSegment.MainStripFront, distance);
    }

    public static void matchProgress() {
      if (lightMode != mode.matchProgress) {
        LEDSegment.MainStrip.clearAnimation();
        LEDSegment.MainStripBack.clearAnimation();
        LEDSegment.MainStripFront.clearAnimation();
        lightMode = mode.matchProgress;
      }
      if (!timerEnabled.get()) return;
      double seconds = RobotStatusTimer.get();
      double multiplier = 0;
      if (robotStatus == RobotStatus.Teleop) {
        multiplier = seconds / 135;
        LEDSegment.MainStrip.setColor(
            new RGBWColor(
                (int) Math.round(green.Red * (1 - multiplier) + red.Red * multiplier),
                (int) Math.round(green.Green * (1 - multiplier) + red.Green * multiplier),
                (int) Math.round(green.Blue * (1 - multiplier) + red.Blue * multiplier)));
        return;
      }
      if (robotStatus == RobotStatus.Autonomous) {
        multiplier = seconds / 30;
        LEDSegment.MainStrip.setColor(
            new RGBWColor(
                (int) Math.round(green.Red * (1 - multiplier) + red.Red * multiplier),
                (int) Math.round(green.Green * (1 - multiplier) + red.Green * multiplier),
                (int) Math.round(green.Blue * (1 - multiplier) + red.Blue * multiplier)));
        return;
      }
      LEDSegment.MainStrip.setColor(green);
    }

    /* 
    public static void twinkle() {
      if (lightMode == mode.twinkle) return;
      lightMode = mode.twinkle;

      LEDSegment.MainStrip.setTwinkleAnimation(green, 0.5, 50);
      LEDSegment.MainStripBack.setTwinkleAnimation(red, 0.5, 30);
      LEDSegment.MainStripFront.setTwinkleAnimation(red, 0.5, 30);
    }

    public static void twinkleOff() {
      if (lightMode == mode.twinkleOff) return;
      lightMode = mode.twinkleOff;

      LEDSegment.MainStrip.setTwinkleOffAnimation(green, 0.5, 50);
      LEDSegment.MainStripBack.setTwinkleOffAnimation(red, 0.5, 30);
      LEDSegment.MainStripFront.setTwinkleOffAnimation(red, 0.5, 30);
    }
    */

    static void updateProgressBar(LEDSegment segment, double distance) {
      if (!progressEnabled.get()) return;
      // This needs to be tested again. I believe that one of the sides wasn't decreasing, but
      // this
      // may have been fixed after the robot was dismantled.
      int delta = findProgressDelta(segment, distance);
      if (delta == 0) return;
      if (!segment.reverseMode) {
        int currentLocation =
            segment.startIndex + segment.progressCount; // Current Location = the light after the
        // last active light
        if (delta > 0) {
          // Turn on forwards
          RGBWColor color = segment.progressOnColor;
          segment.progressSegment(color, currentLocation, delta);
        } else {
          // Turn off backward
          RGBWColor color = segment.progressOffColor;
          segment.progressSegment(color, currentLocation - (-delta), (-delta));
        }
      } else {
        int currentLocation =
            segment.startIndex
                + segment.segmentSize
                - 1
                - segment.progressCount; // Current Location = the light after the last active
        // light
        if (delta > 0) {
          // Turn on backwards
          RGBWColor color = segment.progressOnColor;
          segment.progressSegment(color, currentLocation - delta + 1, delta);
        } else {
          // Turn off forwards
          RGBWColor color = segment.progressOffColor;
          segment.progressSegment(color, currentLocation + 1, (-delta));
        }
      }

      segment.progressCount += delta;
    }

    static int findProgressDelta(LEDSegment segment, double value) {
      // Clamp within the distance range
      value = Math.max(alignToleranceMin, Math.min(alignToleranceMax, value)) - alignToleranceMin;
      // How many lights need to change
      int delta = (int) Math.ceil(value / segment.progressValueDistance) - segment.progressCount;
      return delta;
    }
    // #endregion
  }

  public static enum LEDSegment {
    BatteryIndicator(0, 2, 0, false),
    DriverstationIndicator(2, 2, 1, false),
    ExtraAIndicator(4, 1, -1, false),
    ExtraBIndicator(5, 1, -1, false),
    PivotEncoderIndicator(6, 1, -1, false),
    AllianceIndicator(7, 1, -1, false),
    BuckleIndicator(0, 8, 0, false),
    MainStrip(8, 112, 2, false),
    MainStripBack(8, 62, 3, false, red, yellow),
    MainStripFront(70, 52, 4, true, red, yellow);

    public final int startIndex;
    public final int endIndex;
    public final int segmentSize;
    public final int animationSlot;
    public final boolean reverseMode;

    public int progressCount = 0;
    public final double progressValueDistance;
    public final RGBWColor progressOffColor;
    public final RGBWColor progressOnColor;

    private LEDSegment(int startIndex, int segmentSize, int animationSlot, boolean reverseMode) {
      this.startIndex = startIndex;
      this.segmentSize = segmentSize;
      this.endIndex = startIndex + segmentSize - 1;
      this.animationSlot = animationSlot;
      this.reverseMode = reverseMode;
      this.progressValueDistance = 100 / segmentSize;
      this.progressOffColor = null;
      this.progressOnColor = null;
    }

    private LEDSegment(
        int startIndex,
        int segmentSize,
        int animationSlot,
        boolean reverseMode,
        RGBWColor progressOffColor,
        RGBWColor progressOnColor) {
      this.startIndex = startIndex;
      this.segmentSize = segmentSize;
      this.endIndex = startIndex + segmentSize - 1;
      this.animationSlot = animationSlot;
      this.reverseMode = reverseMode;
      this.progressValueDistance = 100 / segmentSize;
      this.progressOffColor = progressOffColor;
      this.progressOnColor = progressOnColor;
    }

    public void setColor(RGBWColor color) {
      if (candle != null) {
        clearAnimation();
        candle.setControl(new SolidColor(startIndex, endIndex).withColor(color).withUpdateFreqHz(0));
      }
    }
    public void progressSegment(RGBWColor color, int index, int count) {
      if (candle != null) {
        clearAnimation();
        candle.setControl(new SolidColor(index, index + count - 1).withColor(color).withUpdateFreqHz(0));
      }
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
      if (candle != null) {
        setColor(black);
      }
    }

    public void setFlowAnimation(RGBWColor color, double speed, boolean reverse) {
      if (candle != null) {
        boolean modeBoolean = (reverse) ? !reverseMode : reverseMode;
        candle.setControl(new ColorFlowAnimation(startIndex, endIndex).withColor(color).withDirection(modeBoolean ? AnimationDirectionValue.Backward : AnimationDirectionValue.Forward).withFrameRate(speed * 25).withSlot(animationSlot));
      }
    }

    public void setFadeAnimation(RGBWColor color, double speed) {
      if (candle != null) {
        candle.setControl(new SingleFadeAnimation(startIndex, endIndex).withColor(color).withFrameRate(speed * 100).withSlot(animationSlot));
      }
    }

    public void setBandAnimation(RGBWColor color, int size, double speed) {
      if (candle != null) {
        candle.setControl(new LarsonAnimation(startIndex, endIndex).withColor(color).withBounceMode(LarsonBounceValue.Front).withFrameRate(speed * 25).withSlot(animationSlot));
      }
    }
    public void setBandAnimation(RGBWColor color, int size, double speed, LarsonBounceValue bounceValue) {
      if (candle != null) {
        candle.setControl(new LarsonAnimation(startIndex, endIndex).withColor(color).withBounceMode(bounceValue).withFrameRate(speed * 25).withSlot(animationSlot));
      }
    }

    public void setStrobeAnimation(RGBWColor color, double speed) {
      if (candle != null) {
        candle.setControl(new StrobeAnimation(startIndex, endIndex).withColor(color).withFrameRate(speed * 4).withSlot(animationSlot));
      }
    }

    public void setRainbowAnimation(double speed, boolean reverse) {
      if (candle != null) {
        boolean modeBoolean = (reverse) ? !reverseMode : reverseMode;
        candle.setControl(new RainbowAnimation(startIndex, endIndex).withFrameRate(speed * 100).withDirection(modeBoolean ? AnimationDirectionValue.Backward : AnimationDirectionValue.Forward).withSlot(animationSlot));
      }
    }

    public void setFireAnimation(double speed, boolean reverse) {
      if (candle != null) {
        boolean modeBoolean = (reverse) ? !reverseMode : reverseMode;
        candle.setControl(new FireAnimation(startIndex, endIndex).withFrameRate(speed * 100).withDirection(modeBoolean ? AnimationDirectionValue.Backward : AnimationDirectionValue.Forward).withSparking(0.5).withCooling(0.3).withSlot(animationSlot));
      }
    }

    public void setFireOverdriveAnimation(double speed, boolean reverse) {
      if (candle != null) {
        boolean modeBoolean = (reverse) ? !reverseMode : reverseMode;
        candle.setControl(new FireAnimation(startIndex, endIndex).withFrameRate(speed * 100).withDirection(modeBoolean ? AnimationDirectionValue.Backward : AnimationDirectionValue.Forward).withSparking(0.5).withCooling(0.15).withSlot(animationSlot));
      }
    }

    /*
    public void setTwinkleAnimation(RGBWColor color, double speed, double percent) {
      setAnimation(
          new TwinkleAnimation(
              color.Red, color.Green, color.Blue, 0, speed, segmentSize, percent, startIndex));
    }

    public void setTwinkleOffAnimation(
      RGBWColor color, double speed, TwinkleOffAnimation.TwinkleOffPercent divider) {
      setAnimation(
          new TwinkleOffAnimation(
              color.Red, color.Green, color.Blue, 0, speed, segmentSize, divider, startIndex));
    }
    */
  }

  public static class Color {
    public int red;
    public int green;
    public int blue;

    public Color(int red, int green, int blue) {
      this.red = red;
      this.green = green;
      this.blue = blue;
    }

    /**
     * Highly imperfect way of dimming the LEDs. It does not maintain color or accurately adjust
     * perceived brightness.
     *
     * @param dimFactor
     * @return The dimmed color
     */
    public Color dim(double dimFactor) {
      int newRed = (int) (ensureRange(red * dimFactor, 0, 200));
      int newGreen = (int) (ensureRange(green * dimFactor, 0, 200));
      int newBlue = (int) (ensureRange(blue * dimFactor, 0, 200));

      return new Color(newRed, newGreen, newBlue);
    }
  } 

  private static RGBWColor toRGBWColor(Color color) {
    return new RGBWColor((int) color.red, (int) color.green, (int) color.blue).scaleBrightness(1);
  }
  
  private static double ensureRange(double value, double low, double upper) {
    return Math.max(low, Math.min(upper, value));
  }

  public static void disableLEDs() {
    setBrightness(0);
  }

  public static void enableLEDs() {
    setBrightness(1.0);
  }
}