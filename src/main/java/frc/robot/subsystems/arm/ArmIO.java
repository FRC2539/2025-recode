package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

    public void updateInputs(ArmIOInputs inputs);

    @AutoLog
    public class ArmIOInputs {
        public double position = 0.0; 
        public double voltage = 0.0; 
        public double velocity = 0.0; 
        public double current = 0.0;
        public double temperature = 0.0;
        public boolean throughboreEncoderConnection = false;
        public double throughboreEncoderPos = 0.0;
    }

    public void setVoltage(double voltage);
    public void setPosition(double position);
    
} 
