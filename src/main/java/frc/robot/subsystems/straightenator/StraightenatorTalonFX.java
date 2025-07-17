package frc.robot.subsystems.straightenator;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.StraightenatorConstants;

public class StraightenatorTalonFX implements StraightenatorIO {

    private final TalonFX leftWheelMotor =
        new TalonFX(StraightenatorConstants.leftWheelMotorId, StraightenatorConstants.leftWheelMotorCanbus);

    private final TalonFX rightWheelMotor = 
        new TalonFX(StraightenatorConstants.rightWheelMotorId, StraightenatorConstants.rightWheelMotorCanbus);

    private DigitalInput straightenatorSensor = new DigitalInput(StraightenatorConstants.straightenatorSensorPort);
    private DigitalInput cradleSensor = new DigitalInput(StraightenatorConstants.cradleSensorPort);
    
    @Override
    public void updateInputs(StraightenatorIOInputs inputs) {
        inputs.leftVoltage = leftWheelMotor.getMotorVoltage().refresh().getValueAsDouble();
        inputs.rightVoltage = rightWheelMotor.getMotorVoltage().refresh().getValueAsDouble();
        inputs.straightenatorSensor = straightenatorSensor.get();
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
    public int straightenatorSensorValue() {
        return straightenatorSensor.getValue();
    }
    

}