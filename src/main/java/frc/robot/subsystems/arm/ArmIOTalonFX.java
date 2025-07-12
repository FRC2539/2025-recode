package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.constants.ArmConstants;

public class ArmIOTalonFX implements ArmIO {

  private double positionSetpoint = 0;

  private final TalonFX armMotor =
      new TalonFX(ArmConstants.armMotorID, ArmConstants.armMotorCanbus);

  private final DutyCycleEncoder armEncoder = new DutyCycleEncoder(ArmConstants.encoderID);

  private MotionMagicVoltage magicVoltage = new MotionMagicVoltage(positionSetpoint);

  public ArmIOTalonFX() {
    magicVoltage.Slot = 0;

    TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withMotionMagic(ArmConstants.motionMagicConfigs)
            .withSlot0(ArmConstants.slot0Configs)
            .withCurrentLimits(ArmConstants.currentLimit);

    armMotor.getConfigurator().apply(config);

    armMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.position = armEncoder.get();
    inputs.voltage = armMotor.getMotorVoltage().refresh().getValueAsDouble();
    inputs.temperature = armMotor.getDeviceTemp().getValueAsDouble();
  }
  ;

  @Override
  public void setPosition(double position) {
    armMotor.setControl(magicVoltage.withPosition(position));
  }
  ;

  @Override
  public void setVoltage(double voltage) {
    armMotor.setVoltage(voltage);
  }
  ;
}
