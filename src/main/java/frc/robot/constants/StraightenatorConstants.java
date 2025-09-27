package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

public class StraightenatorConstants {
  public static int leftWheelMotorId = 16;
  public static String leftWheelMotorCanbus = "rio";
  public static int straightenatorSensorPort = 3;
  public static final CurrentLimitsConfigs leftwheelscurrentlimit = new CurrentLimitsConfigs();
  public static int ejectLeftVoltage = -3;

  public static int rightWheelMotorId = 17;
  public static String rightWheelMotorCanbus = "rio";
  public static int cradleSensorPort = 6;
  public static final CurrentLimitsConfigs rightwheelscurrentlimit = new CurrentLimitsConfigs();
  public static int ejectRightVoltage = 0;
}
