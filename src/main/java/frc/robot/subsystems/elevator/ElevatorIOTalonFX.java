package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.ElevatorConstants;

public class ElevatorIOTalonFX implements ElevatorIO {
  private final TalonFX elevatorMotor =
      new TalonFX(ElevatorConstants.elevatorMotorId, ElevatorConstants.elevatorMotorCanbus);

  private final MotionMagicVoltage magicVoltage = new MotionMagicVoltage(0); // tune

  public ElevatorIOTalonFX() {
    elevatorMotor.setPosition(0);

    magicVoltage.Slot = 0;

    SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs =
        new SoftwareLimitSwitchConfigs() // tune
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(ElevatorConstants.upperLimit)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(ElevatorConstants.lowerLimit);

    // FeedbackConfigs feedbackConfigs = new FeedbackConfigs().withSensorToMechanismRatio(0); //tune

    TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withSoftwareLimitSwitch(softwareLimitSwitchConfigs)
            .withSlot0(ElevatorConstants.slotconfig)
            .withMotionMagic(ElevatorConstants.motionMagic)
            .withCurrentLimits(ElevatorConstants.currentLimitConfig);
    // .withFeedback(FeedbackConfigs);

    elevatorMotor.getConfigurator().apply(config);

    elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void updateInputs(ElevatorIOInputs inputs) {

    inputs.position = elevatorMotor.getPosition().refresh().getValueAsDouble();
    inputs.voltage = elevatorMotor.getMotorVoltage().refresh().getValueAsDouble();
    inputs.speed = elevatorMotor.getVelocity().refresh().getValueAsDouble();
    inputs.temperature = elevatorMotor.getDeviceTemp().refresh().getValueAsDouble();
    inputs.current = elevatorMotor.getStatorCurrent().refresh().getValueAsDouble();
  }

  public void setVoltage(double voltage) {
    elevatorMotor.setVoltage(voltage);
  }

  public void setPosition(double position) {
    if (position > ElevatorConstants.upperLimit) position = ElevatorConstants.upperLimit;

    if (position < ElevatorConstants.lowerLimit) position = ElevatorConstants.lowerLimit;

    elevatorMotor.setControl(magicVoltage.withPosition(position));
  }
}
