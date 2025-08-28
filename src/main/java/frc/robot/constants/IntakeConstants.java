package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

public class IntakeConstants {
  public static final int pivotMotorId = 100; // TODO: Set the correct id
  public static final String pivotMotorCanBus = ""; // TODO: Set the correct CAN bus name
  public static final CurrentLimitsConfigs pivotCurrentLimit =
      new CurrentLimitsConfigs(); // TODO: set the right current limit

  public static final int wheelsMotorId = 100; // TODO: Set the correct CAN bus name
  public static final String wheelsMotorCanBus = ""; // TODO: Set the correct CAN bus name
  public static final CurrentLimitsConfigs wheelsCurrentLimit =
      new CurrentLimitsConfigs(); // TODO: Set the correct current limit

  public static final double intakeUpPosition = 20;
  public static final double intakeDownPosition = 60;
  public static final double intakeVoltage = 12;
}
