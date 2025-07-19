package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.ElevatorConstants;

public class ElevatorIOTalonFX implements ElevatorIO {
  private final TalonFX elevatorMotor =
      new TalonFX(ElevatorConstants.elevatorMotorId, ElevatorConstants.elevatorMotorCanbus);

  private final TalonFX elevatorMotorFollower =
      new TalonFX(
          ElevatorConstants.elevatorMotorFollowerId, ElevatorConstants.elevatorMotorFollowerCanbus);


  private double targetPosition = 0;
  private final MotionMagicVoltage magicVoltage = new MotionMagicVoltage(0); // tune

  public ElevatorIOTalonFX() {
    elevatorMotorFollower.setControl(new Follower(elevatorMotor.getDeviceID(), true));


    elevatorMotor.setPosition(0);
    elevatorMotorFollower.setPosition(0);

    magicVoltage.Slot = 0;

    SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs =
        new SoftwareLimitSwitchConfigs() // tune
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(ElevatorConstants.upperLimit)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(ElevatorConstants.lowerLimit);

    TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withSoftwareLimitSwitch(softwareLimitSwitchConfigs)
            .withSlot0(ElevatorConstants.slot0Configs)
            .withMotionMagic(ElevatorConstants.motionMagicConfigs)
            .withCurrentLimits(ElevatorConstants.currentLimit);
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

    targetPosition = position;

    elevatorMotor.setControl(magicVoltage.withPosition(position));
  }

  @Override 
  public boolean isAtSetpoint() {
    return Math.abs(targetPosition - elevatorMotor.getPosition().refresh().getValueAsDouble()) < 1;
  }
}
