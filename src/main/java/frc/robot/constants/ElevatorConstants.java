package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

public class ElevatorConstants {

  public static final Slot0Configs slot0Configs = new Slot0Configs()
      // .withKA(0)
      // .withKD(0)
      // .withKG(0)
      // .withKI(0)
      // .withKP(0)
      // .withKS(0)
      // .withKV(0)
      // .withGravityType(GravityTypeValue.Elevator_Static);
      ;
  public static final MotionMagicConfigs motionMagicConfigs =
      new MotionMagicConfigs()
          .withMotionMagicAcceleration(190 / .3) // these are guesses, come back here
          .withMotionMagicCruiseVelocity(190) // also guess
          .withMotionMagicJerk(0);

  // Set correct IDs
  public static final int elevatorMotorId = 10;
  public static final int elevatorMotorFollowerId = 9;

  public static final String elevatorMotorCanbus = "CANivore";
  public static final String elevatorMotorFollowerCanbus = "CANivore";

  public static final CurrentLimitsConfigs currentLimit = new CurrentLimitsConfigs();

  // Set correct limits
  public static final double lowerLimit = 0;
  public static final double upperLimit = 500;
}
