package frc.robot.subsystems.gripper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.constants.GripperConstants;
import frc.robot.constants.GripperConstants.Piece;

public class GripperIOTalonFX implements GripperIO {

  private ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
  private ColorMatch colorMatcher = new ColorMatch();

  private TalonFX gripperMotor = new TalonFX(GripperConstants.gripperMotorID);

  public GripperIOTalonFX() {
    gripperMotor.setPosition(0);
    colorMatcher.addColorMatch(new Color(255, 255, 255));
    colorMatcher.addColorMatch(new Color(0, 0, 255));

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
    Color color = colorSensor.getColor();
    ColorMatchResult matchResult = colorMatcher.matchClosestColor(color);
    return matchResult.confidence > GripperConstants.targetSensorConfidence;
  }

  @Override 
  public Piece getPieceType() {
    Color color = colorSensor.getColor();
    ColorMatchResult matchResult = colorMatcher.matchClosestColor(color);
    
    if (!hasPiece()) {
      return Piece.NONE;
    }

    if (matchResult.color == new Color(255, 255, 255)) {
      return Piece.CORAL;
    } else {
      return Piece.ALGAE;
    }
  }
}
