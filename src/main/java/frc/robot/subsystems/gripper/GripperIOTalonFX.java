package frc.robot.subsystems.gripper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.constants.GripperConstants;

public class GripperIOTalonFX implements GripperIO{
    
    private CANrange distanceSensor = new CANrange(GripperConstants.gripperSensorID);

    private TalonFX gripperMotor = new TalonFX(GripperConstants.gripperMotorID);

    public GripperIOTalonFX(){
        gripperMotor.setPosition(0);

        TalonFXConfiguration talonConfig = new TalonFXConfiguration();

        gripperMotor.getConfigurator().apply(talonConfig);
    }

    public void updateInputs(GripperIOInputs inputs){
        inputs.voltage = gripperMotor.getMotorVoltage().refresh().getValueAsDouble();
        inputs.speed = gripperMotor.getVelocity().refresh().getValueAsDouble();
        inputs.hasPiece = distanceSensor.getIsDetected().getValue();
    }
    
    public void setVoltage(double voltage){
        gripperMotor.setVoltage(voltage);
    }

    public boolean hasPiece(){
        return distanceSensor.getIsDetected().getValue();
    }
}

