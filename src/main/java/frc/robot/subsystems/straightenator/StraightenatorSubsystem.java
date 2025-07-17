package frc.robot.subsystems.straightenator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.straightenator.StraightenatorIO.StraightenatorIOInputs;
import org.littletonrobotics.junction.Logger;

public class StraightenatorSubsystem extends SubsystemBase {
    
    private StraightenatorIO straightenatorIO;
    private StraightenatorIOInputsAutoLogged straightenatorInputs = new StraightenatorIOInputsAutoLogged();

    public StraightenatorSubsystem(StraightenatorIO straightenatorIO) {
        this.straightenatorIO = straightenatorIO;
    }

    public Command setWheelVoltage(double voltage){
        return Commands.runOnce(() -> {straightenatorIO.setVoltage(voltage);});
    }

    public Command intakeUntilPiece() {
        return Commands.either(return setVoltage(setWheelVoltage));
    }

    public boolean hasPiece() {
        return StraightenatorIOInputs.straightenatorSensor;
    }

    public boolean intaking() {
        return StraightenatorIOInputs.cradleSensor;
    }

    @Override
    public void periodic() {
         
        straightenatorIO.updateInputs(straightenatorInputs);

        Logger.processInputs("RealOutputs/Elevator", straightenatorInputs);
    
    }

}
