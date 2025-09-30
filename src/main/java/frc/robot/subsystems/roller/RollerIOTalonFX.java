package frc.robot.subsystems.roller;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.constants.IntakeConstants;

public class RollerIOTalonFX implements RollerIO {


  private final TalonFX wheelsMotor =
      new TalonFX(IntakeConstants.wheelsMotorId, IntakeConstants.wheelsMotorCanBus);


  public RollerIOTalonFX() {

    TalonFXConfiguration wheelsConfig =
        new TalonFXConfiguration().withCurrentLimits(IntakeConstants.wheelsCurrentLimit);

    wheelsMotor.getConfigurator().apply(wheelsConfig);
    wheelsMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.wheelsVoltage = wheelsMotor.getMotorVoltage().refresh().getValueAsDouble();
  }

  @Override
  public void setWheelsVoltage(double voltage) {
    wheelsMotor.setVoltage(voltage);
  }
}
