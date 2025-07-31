package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class AlignToReef extends Command {
    private Drive drive;

    private DoubleSupplier xVelocity;
    private DoubleSupplier yVelocity;

    private PIDController thetaController = new PIDController(0, 0, 0);
    private PIDController yController = new PIDController(0, 0, 0);

    private Pose2d targetPose;
    private double offset;
    private Rotation2d rotationOffset;

    private Timer slewerTimer = new Timer();

    public AlignToReef(
            Drive drive,
            DoubleSupplier xVelocity,
            DoubleSupplier yVelocity,
            double alignmentOffset,
            Pose2d alignmentPose,
            Rotation2d rotationOffset) {
        this.drive = drive;
        this.xVelocity = xVelocity;
        this.yVelocity = yVelocity;
        this.offset = alignmentOffset;
        this.targetPose = alignmentPose;
        this.rotationOffset = rotationOffset;
    }

    @Override
    public void initialize() {
        slewerTimer.restart();
        Pose2d tCurrentPose = drive.getPose();

        Logger.recordOutput("/AutoAlign/CurrentPose", tCurrentPose);
        Logger.recordOutput("/AutoAlign/TargetPose", targetPose);
        Logger.recordOutput("/AutoAlign/Offset", offset);

        thetaController.setSetpoint(rotationOffset.getRadians());
        yController.setSetpoint(offset);
        thetaController.enableContinuousInput(0, 2 * Math.PI);
        thetaController.setTolerance(Units.degreesToRadians(0.5));
        yController.setTolerance(Units.inchesToMeters(0.2));
    }

    @Override
    public void execute() {
        // double currentHeading = drivetrain.getState().Pose.getRotation().getRadians();
        Pose2d currentPose = drive.getPose().Pose;
        
        Transform2d offset = currentPose.minus(targetPose);

        double thetaVelocity = thetaController.calculate(offset.getRotation().getRadians());
        if (thetaController.atSetpoint()) {
            thetaVelocity = 0;
        }
        double yVelocityController = yController.calculate(offset.getY());
        if (yController.atSetpoint()) {
            yVelocityController = 0;
        }

        Rotation2d tagRotation = targetPose.getRotation();

        ChassisSpeeds driverCommandedVelocities =
                new ChassisSpeeds(xVelocity.getAsDouble(), yVelocity.getAsDouble(), 0);

        ChassisSpeeds fieldCommandedVelocities =
                ChassisSpeeds.fromRobotRelativeSpeeds(
                        driverCommandedVelocities, drive.getOperatorForwardDirection());

        ChassisSpeeds tagRelativeCommandedVelocities =
                ChassisSpeeds.fromFieldRelativeSpeeds(fieldCommandedVelocities, tagRotation);

        tagRelativeCommandedVelocities.vyMetersPerSecond = yVelocityController;
        tagRelativeCommandedVelocities.omegaRadiansPerSecond = thetaVelocity;

        ChassisSpeeds fieldRelativeSpeeds =
                ChassisSpeeds.fromRobotRelativeSpeeds(tagRelativeCommandedVelocities, tagRotation);

    }

    @Override
    public boolean isFinished() {
        return false;

    }
}