package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

public class ArmConstants {
  public static final Slot0Configs slot0Configs =
      new Slot0Configs()
          .withKS(0)
          .withKV(0)
          .withKA(0)
          .withKD(0)
          .withKI(0)
          .withKP(3.5)
          .withKG(0.001);
  public static final MotionMagicConfigs motionMagicConfigs =
      new MotionMagicConfigs()
          .withMotionMagicAcceleration(1000)
          .withMotionMagicCruiseVelocity(500)
          .withMotionMagicJerk(8000); // 10000

  public static final int armMotorID = 11;
  public static final int encoderID = 31;
  public static final String armMotorCanbus = "rio";
  public static final CurrentLimitsConfigs currentLimit = new CurrentLimitsConfigs();
  public static final double armSetpointTolerance = 0.75;
}
