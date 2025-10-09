package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class leds extends SubsystemBase {
  private final CANdle m_candle;

  public leds() {
    m_candle = new CANdle(18, "CANivore");
    CANdleConfiguration candleConfiguration = new CANdleConfiguration();

    candleConfiguration.stripType = LEDStripType.RGB;
    candleConfiguration.brightnessScalar = 1.0;
    candleConfiguration.vBatOutputMode = VBatOutputMode.Modulated;
    ErrorCode code = m_candle.configAllSettings(candleConfiguration, 4000);

    // if (code != ErrorCode.OK) {
    //   throw new ExceptionInInitializerError("Failed to configure CANdle: " + code.toString());
    // }
    m_candle.configBrightnessScalar(1);
    setDefaultCommand(Commands.run(() -> m_candle.setLEDs(255, 102, 0), this)); // orange!
  }
}
