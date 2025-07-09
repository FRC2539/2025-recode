package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.ElevatorConstants;

public class ElevatorSubsystem implements ElevatorIO {

  private TalonFX elevatorLeaderMotor = new TalonFX(ElevatorConstants.elevatorMotorId); // Leader
  private TalonFX elevatorFollowerMotor =
      new TalonFX(ElevatorConstants.elevatorMotorFollowerId); // Follower

  public double target = 0;
  final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

  public ElevatorSubsystem() {
    elevatorLeaderMotor.setPosition(0);
    elevatorFollowerMotor.setPosition(0);

    elevatorFollowerMotor.setControl(new Follower(elevatorLeaderMotor.getDeviceID(), true));
    TalonFXConfiguration rightMotorConfigs = new TalonFXConfiguration();
    // leftMotorConfigs.MotorOutput.OpenLoopRamp = 0.2;
    rightMotorConfigs.MotorOutput.PeakForwardDutyCycle = 0;
    rightMotorConfigs.MotorOutput.PeakReverseDutyCycle = 0;

    var talonFXConfigs = new TalonFXConfiguration();

    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0;
    slot0Configs.kV = 0;
    slot0Configs.kA = 0;
    slot0Configs.kP = 0;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;

    talonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    talonFXConfigs.MotionMagic.MotionMagicAcceleration =
        160; // Target acceleration of 160 rps/s (0.5 seconds)
    talonFXConfigs.MotionMagic.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    elevatorLeaderMotor.getConfigurator().apply(talonFXConfigs);
    elevatorFollowerMotor.getConfigurator().apply(talonFXConfigs);
    elevatorLeaderMotor.setNeutralMode(NeutralModeValue.Brake);
    elevatorFollowerMotor.setNeutralMode(NeutralModeValue.Brake);
    elevatorFollowerMotor.getConfigurator().apply(rightMotorConfigs);
  }

  public void updateInputs(ElevatorIOInputs inputs) {

    inputs.position = elevatorLeaderMotor.getPosition().refresh().getValueAsDouble();
    inputs.voltage = elevatorLeaderMotor.getMotorVoltage().refresh().getValueAsDouble();
    inputs.speed = elevatorLeaderMotor.getVelocity().refresh().getValueAsDouble();
    inputs.temperature = elevatorLeaderMotor.getDeviceTemp().getValueAsDouble();
    inputs.current = elevatorLeaderMotor.getStatorCurrent().getValueAsDouble();

    MotionMagicVoltage goal = m_request.withPosition(target).withSlot(0);
    elevatorLeaderMotor.setControl(goal);
    // elevatorRightMotor.setControl(goal);

    // System.out.println("target: "+target);
    // System.out.println("left: "+elevatorLeftMotor.getPosition().refresh().getValueAsDouble());
    // System.out.println("right: "+elevatorRightMotor.getPosition().refresh().getValueAsDouble());
  }

  public void setVoltage(double voltage) {
    elevatorLeaderMotor.setVoltage(voltage);
    elevatorFollowerMotor.setVoltage(-voltage);
  }

  public void setPosition(double position) {
    this.target = position;

    MotionMagicVoltage goal = m_request.withPosition(position).withEnableFOC(false).withSlot(0);

    elevatorLeaderMotor.setControl(goal);
  }

  public void resetPosition() {
    elevatorLeaderMotor.setPosition(0);
  }
}
