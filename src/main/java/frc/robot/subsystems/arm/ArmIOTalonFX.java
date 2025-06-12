package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.constants.ArmConstants;

public class ArmIOTalonFX implements ArmIO {
    private final TalonFX armPivotMotor =
            new TalonFX(
                    ArmConstants.ARM_MOTOR_ID,
                    ArmConstants.ARM_CANBUS);

    private final DutyCycleEncoder throughboreEncoder =
            new DutyCycleEncoder(ArmConstants.ARM_THROUGHBORE_ENCODER_ID, 2 * Math.PI, 0);

    private double lastVoltage = 0;

    public ArmIOTalonFX() {
        armPivotMotor.setPosition(0);

        TalonFXConfigurator talonConfig = armPivotMotor.getConfigurator();

        talonConfig.apply(
                new TalonFXConfiguration()
                        .withCurrentLimits(ArmConstants.currentLimitConfigs)
                        .withMotorOutput(
                                new MotorOutputConfigs()
                                        .withInverted(InvertedValue.Clockwise_Positive)));

        armPivotMotor.setNeutralMode(NeutralModeValue.Brake);
    }



    public void updateInputs(ArmIOInputs inputs) {

        inputs.position = armPivotMotor.getPosition().getValueAsDouble();
        inputs.current = armPivotMotor.getStatorCurrent().getValueAsDouble();
        inputs.voltage = armPivotMotor.getMotorVoltage().getValueAsDouble();
        inputs.velocity = armPivotMotor.getVelocity().getValueAsDouble();
        inputs.temperature = armPivotMotor.getDeviceTemp().getValueAsDouble();
        inputs.throughboreEncoderPos = (throughboreEncoder.get() * -1 + 2 * Math.PI) - 3.065;

        if (inputs.throughboreEncoderPos >= ArmConstants.upperLimit && lastVoltage > 0) {
            lastVoltage = 0;
        }
        if (inputs.throughboreEncoderPos <= ArmConstants.lowerLimit && lastVoltage < 0) {
            lastVoltage = 0;
        }
        armPivotMotor.setVoltage(lastVoltage);
    }

    public void setVoltage(double voltage) {
        lastVoltage = voltage;
    }

    public void setPosition(double position) {
    }
}