package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.robot.constants.ArmConstants;

public class ArmIOTalonFX implements ArmIO {

  private double positionSetpoint = 0;

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

    // config.Feedback.FeedbackRemoteSensorID = armEncoder.getDeviceID();
    // config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

    // armMotor.getConfigurator().apply(config);

    /* Configure CANcoder to zero the magnet appropriately */
    CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
    // cc_cfg.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    cc_cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    cc_cfg.MagnetSensor.MagnetOffset = 0.4;
    armEncoder.getConfigurator().apply(cc_cfg);

    config.Feedback.FeedbackRemoteSensorID = armEncoder.getDeviceID();
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
    config.Feedback.SensorToMechanismRatio = 1.0;
    config.Feedback.RotorToSensorRatio = 63.2;

    armMotor.getConfigurator().apply(config);

    armMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {

    inputs.position = armMotor.getPosition().refresh().getValueAsDouble();

    inputs.voltage = armMotor.getMotorVoltage().refresh().getValueAsDouble();
    inputs.temperature = armMotor.getDeviceTemp().getValueAsDouble();

    inputs.isAtSetpoint = isAtSetpoint();
  }

  @Override
  public void setPosition(double position) {

    positionSetpoint = position;
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
