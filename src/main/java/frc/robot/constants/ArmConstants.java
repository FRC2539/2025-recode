package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

public class ArmConstants {

    public static final int ARM_MOTOR_ID = 10;
    public static final String ARM_CANBUS = "rio";

    public static final int ARM_THROUGHBORE_ENCODER_ID = 1;

    public static final CurrentLimitsConfigs currentLimitConfigs = new CurrentLimitsConfigs();
    public static final double ARM_KP = 0;
    public static final double ARM_KD = 0; 
    public static final double ARM_KI = 0; 

    public static final double ARM_TOLERANCE = 0.0;

    public static final double upperLimit = 0;
    public static final double lowerLimit = 0;
}