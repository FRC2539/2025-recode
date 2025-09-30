package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.constants.IntakeConstants;

public class IntakeIOTalonFX implements IntakeIO {

  private double positionSetpoint = 0;

  private final TalonFX pivotMotor =
      new TalonFX(IntakeConstants.pivotMotorId, IntakeConstants.pivotMotorCanBus);

  // private final TalonFX wheelsMotor =
  //     new TalonFX(IntakeConstants.wheelsMotorId, IntakeConstants.wheelsMotorCanBus);

  private PIDController pivotController = new PIDController(0.3, 0, 0);

  public IntakeIOTalonFX() {
    TalonFXConfiguration pivotConfig =
        new TalonFXConfiguration().withCurrentLimits(IntakeConstants.pivotCurrentLimit);

    // TalonFXConfiguration wheelsConfig =
    //     new TalonFXConfiguration().withCurrentLimits(IntakeConstants.wheelsCurrentLimit);

    pivotMotor.getConfigurator().apply(pivotConfig);
    // wheelsMotor.getConfigurator().apply(wheelsConfig);

    pivotMotor.setNeutralMode(NeutralModeValue.Brake);
    // wheelsMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.pivotVoltage = pivotMotor.getMotorVoltage().refresh().getValueAsDouble();
    inputs.pivotPosition = pivotMotor.getPosition().refresh().getValueAsDouble();
    // inputs.wheelsVoltage = wheelsMotor.getMotorVoltage().refresh().getValueAsDouble();

    pivotMotor.setVoltage(pivotController.calculate(inputs.pivotPosition));
  }

  // @Override
  // public void setWheelsVoltage(double voltage) {
  //   wheelsMotor.setVoltage(voltage);
  // }

  @Override
  public void setPivotPosition(double position) {
    positionSetpoint = position;
    pivotController.setSetpoint(positionSetpoint);
  }

  @Override
  public boolean isAtSetpoint() {
    return Math.abs(pivotMotor.getPosition().refresh().getValueAsDouble() - positionSetpoint)
        < IntakeConstants.intakeSetpointTolerance;
  }
}
