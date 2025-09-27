package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

public class ArmConstants {
  public static final Slot0Configs slot0Configs =
      new Slot0Configs().withKS(0).withKV(0).withKA(0).withKD(0).withKI(0).withKP(0);
  public static final MotionMagicConfigs motionMagicConfigs =
      new MotionMagicConfigs()
          .withMotionMagicAcceleration(0)
          .withMotionMagicCruiseVelocity(0)
          .withMotionMagicJerk(0);

  // TODO: get arm motor id + encoder ID
  public static final int armMotorID = 11;
  public static final int encoderID = 0;
  public static final String armMotorCanbus = "rio";
  public static final CurrentLimitsConfigs currentLimit = new CurrentLimitsConfigs();
  public static final double armSetpointTolerance = 0.2;
}
