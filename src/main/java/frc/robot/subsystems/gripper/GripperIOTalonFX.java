package frc.robot.subsystems.gripper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import frc.robot.constants.GripperConstants;

public class GripperIOTalonFX implements GripperIO {

    //GET REV ROBOTICS VENDOR DEPENDENCY FOR COLOR SENSOR V3 AND FIX SENSOR

  private I2C i2c = new I2C(Port.kOnboard, GripperConstants.gripperSensorAddress);

  private TalonFX gripperMotor = new TalonFX(GripperConstants.gripperMotorID);

  public GripperIOTalonFX() {
    gripperMotor.setPosition(0);

    TalonFXConfiguration talonConfig = new TalonFXConfiguration();

    gripperMotor.getConfigurator().apply(talonConfig);
  }

  public void updateInputs(GripperIOInputs inputs) {
    inputs.voltage = gripperMotor.getMotorVoltage().refresh().getValueAsDouble();
    inputs.speed = gripperMotor.getVelocity().refresh().getValueAsDouble();
  }

  public void setVoltage(double voltage) {
    gripperMotor.setVoltage(voltage);
  }

  public boolean hasPiece() {
    return true;
  }
}
