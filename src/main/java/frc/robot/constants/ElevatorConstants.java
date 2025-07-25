package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

public class ElevatorConstants {

  public static final Slot0Configs slot0Configs =
      new Slot0Configs()
          .withKS(0)
          .withKV(0)
          .withKA(0)
          .withKD(0)
          .withKI(0)
          .withKP(0); // TODO: Set the correct values
  public static final MotionMagicConfigs motionMagicConfigs =
      new MotionMagicConfigs()
          .withMotionMagicAcceleration(0) // TODO: these are guesses, come back here
          .withMotionMagicCruiseVelocity(0) // TODO: also guess
          .withMotionMagicJerk(0); // TODO: Set the correct value if needed

  // TODO: Set correct IDs
  public static final int elevatorMotorId = 10;
  public static final int elevatorMotorFollowerId = 9;

  public static final String elevatorMotorCanbus = "CANivore";
  public static final String elevatorMotorFollowerCanbus = "CANivore";

  public static final CurrentLimitsConfigs currentLimit = new CurrentLimitsConfigs();

  // TODO: Set correct limits
  public static final double lowerLimit = 0;
  public static final double upperLimit = 500;
}
