package frc.robot.subsystems.arm;

public class ArmIOSim implements ArmIO {

    private double position = 0;
    private double voltage = 0;

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.voltage = voltage;
        inputs.position = position;
    }

    @Override
    public void setPosition(double position) {
        this.position = position;
    }

    @Override
    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }
}
