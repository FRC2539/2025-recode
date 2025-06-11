package frc.robot.subsystems.arm;

public class ArmIOSim implements ArmIO {
    private double position = 0;
    private double voltage = 0;

    public void updateInputs(ArmIOInputs inputs) {
        position += 0.02 * voltage * 0.6;

        inputs.position = position;
        inputs.throughboreEncoderPos = position;
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }

    public void setPosition(double position) {
        this.voltage = voltage;
    }
}