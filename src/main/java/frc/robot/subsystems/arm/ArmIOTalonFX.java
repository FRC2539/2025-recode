package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.ArmConstants;

public class ArmIOTalonFX implements ArmIO {

  private double positionSetpoint = 0.184;

  private final TalonFX armMotor =
      new TalonFX(ArmConstants.armMotorID, ArmConstants.armMotorCanbus);

  private final CANcoder armEncoder = new CANcoder(ArmConstants.encoderID);

  private PositionDutyCycle magicVoltage = new PositionDutyCycle(positionSetpoint);

  public ArmIOTalonFX() {
    magicVoltage.Slot = 0;

    TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withMotionMagic(ArmConstants.motionMagicConfigs)
            .withSlot0(ArmConstants.slot0Configs)
            .withCurrentLimits(ArmConstants.currentLimit);

    config.Feedback.FeedbackRemoteSensorID = armEncoder.getDeviceID();
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
    armMotor.getConfigurator().apply(config);

    armMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {

    inputs.position = armMotor.get();

    inputs.voltage = armMotor.getMotorVoltage().refresh().getValueAsDouble();
    inputs.temperature = armMotor.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setPosition(double position) {

    armMotor.setControl(magicVoltage.withPosition(position));
  }

  @Override
  public void setVoltage(double voltage) {
    armMotor.setVoltage(voltage);
  }

  @Override
  public boolean isAtSetpoint() {
    return Math.abs(armMotor.getPosition().refresh().getValueAsDouble() - positionSetpoint)
        < ArmConstants.armSetpointTolerance;
  }
}
