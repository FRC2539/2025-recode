package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

public class IntakeConstants {
  public static final int pivotMotorId = 13; // TODO: Set the correct id
  public static final String pivotMotorCanBus = "rio"; // TODO: Set the correct CAN bus name
  public static final CurrentLimitsConfigs pivotCurrentLimit =
      new CurrentLimitsConfigs(); // TODO: set the right current limit

  public static final int wheelsMotorId = 14; // TODO: Set the correct CAN bus name
  public static final String wheelsMotorCanBus = "rio"; // TODO: Set the correct CAN bus name
  public static final CurrentLimitsConfigs wheelsCurrentLimit =
      new CurrentLimitsConfigs(); // TODO: Set the correct current limit

  public static final double intakeUpPosition = -7;
  public static final double intakeDownPosition = -26.51;
  public static final double intakeMidPosition = -7;
  public static final double intakeVoltage = 6;

  public static final double intakeSetpointTolerance = 1;
}
