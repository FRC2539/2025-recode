package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;

public class ElevatorConstants {
  public static final Slot0Configs slotconfig =
      new Slot0Configs() // tune
          .withKA(0)
          .withKD(0)
          .withKG(0)
          .withKI(0)
          .withKP(0)
          .withKS(0)
          .withKV(0)
          .withGravityType(GravityTypeValue.Elevator_Static);

  public static final MotionMagicConfigs motionMagic =
      new MotionMagicConfigs() // tune
          .withMotionMagicAcceleration(null)
          .withMotionMagicCruiseVelocity(null)
          .withMotionMagicJerk(null)
          .withMotionMagicExpo_kA(null)
          .withMotionMagicExpo_kV(null);

  public static final int elevatorMotorId = 9; // tune

  public static final String elevatorMotorCanbus = "CANivore";

  public static final CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs();

  public static final double lowerLimit = 0; // tune

  public static final double upperLimit = 500; // tune
}
