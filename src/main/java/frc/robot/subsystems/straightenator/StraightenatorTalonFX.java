package frc.robot.subsystems.straightenator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.StraightenatorConstants;
import org.littletonrobotics.junction.AutoLogOutput;

public class StraightenatorTalonFX implements StraightenatorIO {

  private final TalonFX leftWheelMotor =
      new TalonFX(
          StraightenatorConstants.leftWheelMotorId, StraightenatorConstants.leftWheelMotorCanbus);

  private final TalonFX rightWheelMotor =
      new TalonFX(
          StraightenatorConstants.rightWheelMotorId, StraightenatorConstants.rightWheelMotorCanbus);

  private DigitalInput straightenatorSensor =
      new DigitalInput(StraightenatorConstants.straightenatorSensorPort);
  private DigitalInput cradleSensor = new DigitalInput(StraightenatorConstants.cradleSensorPort);

  @Override
  public void updateInputs(StraightenatorIOInputs inputs) {
    inputs.leftVoltage = leftWheelMotor.getMotorVoltage().refresh().getValueAsDouble();
    inputs.rightVoltage = rightWheelMotor.getMotorVoltage().refresh().getValueAsDouble();
    inputs.isStraight = straightenatorSensor.get();
    inputs.isCradled = cradleSensor.get();
  }

  public StraightenatorTalonFX() {
    TalonFXConfiguration pivotConfig =
        new TalonFXConfiguration()
            .withCurrentLimits(StraightenatorConstants.leftwheelscurrentlimit);

    TalonFXConfiguration wheelsConfig =
        new TalonFXConfiguration()
            .withCurrentLimits(StraightenatorConstants.rightwheelscurrentlimit);

    leftWheelMotor.getConfigurator().apply(pivotConfig);
    rightWheelMotor.getConfigurator().apply(wheelsConfig);

    leftWheelMotor.setNeutralMode(NeutralModeValue.Brake);
    rightWheelMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void setVoltage(double voltage) {
    setLeftMotorVoltage(voltage);
    setRightMotorVoltage(voltage);
  }

  @Override
  public void setLeftMotorVoltage(double voltage) {
    leftWheelMotor.setVoltage(voltage);
  }

  @Override
  public void setRightMotorVoltage(double voltage) {
    rightWheelMotor.setVoltage(voltage);
  }

  @AutoLogOutput
  public boolean straightenatorSensorValue() {
    return !straightenatorSensor.get();
  }

  public boolean cradleSensorValue() {
    return !cradleSensor.get();
  }
}
