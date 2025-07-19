package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

public class StraightenatorConstants {
  public static int leftWheelMotorId = 101;
  public static String leftWheelMotorCanbus = "";
  public static int straightenatorSensorPort = 0;
  public static final CurrentLimitsConfigs leftwheelscurrentlimit = new CurrentLimitsConfigs();

  public static int rightWheelMotorId = 102;
  public static String rightWheelMotorCanbus = "";
  public static int cradleSensorPort = 0;
  public static final CurrentLimitsConfigs rightwheelscurrentlimit = new CurrentLimitsConfigs();
}
