package frc.robot.subsystems.straightenator;

import org.littletonrobotics.junction.AutoLog;

public interface StraightenatorIO {
    
    public void updateInputs(StraightenatorIOInputs inputs);

    @AutoLog
    public class StraightenatorIOInputs {
        double leftVoltage = 0;
        double rightVoltage = 0;
        boolean isStraight = false;
        boolean isCradled = false;

    }

    public boolean straightenatorSensor = false;

    public boolean cradleSensor = false;

    public void setVoltage(double voltage);

    public void setLeftMotorVoltage(double voltage);

    public void setRightMotorVoltage(double voltage);




}
