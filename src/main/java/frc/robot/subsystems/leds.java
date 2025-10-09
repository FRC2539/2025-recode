// package frc.robot.subsystems;

// import com.ctre.phoenix6.StatusCode;
// import com.ctre.phoenix6.configs.CANdleConfiguration;
// import com.ctre.phoenix6.controls.SolidColor;
// import com.ctre.phoenix6.hardware.CANdle;
// import com.ctre.phoenix6.signals.RGBWColor;
// import com.ctre.phoenix6.signals.StripTypeValue;
// import com.ctre.phoenix6.signals.VBatOutputModeValue;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class leds extends SubsystemBase {
//   private final CANdle m_candle;

//   public leds() {
//     m_candle = new CANdle(18, "CANivore");
//     CANdleConfiguration candleConfiguration = new CANdleConfiguration();

//     candleConfiguration.CANdleFeatures.VBatOutputMode = VBatOutputModeValue.Modulated;
//     candleConfiguration.LED.BrightnessScalar = 1.0;
//     candleConfiguration.LED.StripType = StripTypeValue.RGB;
//     StatusCode code = m_candle.getConfigurator().apply(candleConfiguration);

//     if (code != StatusCode.OK) {
//       throw new ExceptionInInitializerError("Failed to configure CANdle: " + code.toString());
//     }

//     setDefaultCommand(
//         Commands.run(
//             () ->
//                 m_candle.setControl(new SolidColor(0, 150).withColor(new RGBWColor(255, 255,
// 255))),
//             this));
//   }
// }
