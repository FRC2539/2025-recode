package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {

  private final VisionIO[] io;

  private final VisionIOInputsAutoLogged[] inputs;

  public VisionSubsystem(Supplier<Rotation2d> robotHeading, VisionIO... visionIO) {
    this.io = visionIO;

    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    for (VisionIOInputsAutoLogged input : inputs) {

      Logger.processInputs("RealOutputs/Vision", input);
    }
  }
}
