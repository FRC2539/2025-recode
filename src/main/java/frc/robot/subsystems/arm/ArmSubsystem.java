package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    public ArmIO armIO;
  
    public ArmSubsystem(ArmIO armIO) {
        this.armIO = armIO;
    }

    public void setPosition(double position) {
        armIO.setPosition(position);
    }

    public void setVoltage(double voltage) {
        armIO.setVoltage(voltage);
    }
}