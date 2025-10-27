// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.constants.ClimberConstants;

/** Add your docs here. */
public class ClimberIOTalonFX implements ClimberIO {

  private TalonFX climberMotor =
      new TalonFX(ClimberConstants.climberMotorID, ClimberConstants.climberMotorCanbus);

  private DutyCycleEncoder climberEncoder;

  public ClimberIOTalonFX() {
    climberMotor.setPosition(0);

    climberEncoder = new DutyCycleEncoder(ClimberConstants.encoderID);

    TalonFXConfiguration config = new TalonFXConfiguration();

    climberMotor.getConfigurator().apply(config);

    climberMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void updateInputs(ClimberIOInputs inputs) {
    inputs.position = climberEncoder.get();
    inputs.voltage = climberMotor.getMotorVoltage().refresh().getValueAsDouble();
    inputs.speed = climberMotor.getVelocity().refresh().getValueAsDouble();
    inputs.current = climberMotor.getStatorCurrent().refresh().getValueAsDouble();
  }

  public void setPosition(double position) {
    climberMotor.setPosition(position);
  }

  public void setVoltage(double voltage) {

    if (climberEncoder.get() < 0.822) {
      climberMotor.setVoltage(voltage);
    } else {
      climberMotor.setVoltage(0);
    }
  }
}
