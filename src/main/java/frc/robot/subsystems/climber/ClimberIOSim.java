// Open Source Software; you can modify and/or share it under the ter// the WPILib BSD license file
// in the root directory of this project.

package frc.robot.subsystems.climber;

/** Add your docs here. */
public class ClimberIOSim implements ClimberIO {

  private double voltage = 0.0;
  private double position = 0.0;
  private double speed = 0.0;
  private double current = 0.0;

  public void updateInputs(ClimberIOInputs inputs) {
    inputs.voltage = voltage;
    inputs.position = position;
    inputs.speed = speed;
    inputs.current = current;
  }

  @Override
  public void setVoltage(double voltage) {
    this.voltage = voltage;
  }

  @Override
  public void setPosition(double position) {
    this.position = position;
  }
}
