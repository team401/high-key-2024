package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.PhoenixDriveConstants;
import frc.robot.subsystems.drive.PhoenixDrive;

public class AlignToTarget extends Command {
    private PhoenixDrive drivetrain;
    private Pose2d desiredTargetPose;
    private Pose2d currentPose;
    private PIDController rotationController;
    private final NetworkTableInstance inst;

    public AlignToTarget(PhoenixDrive drivetrain, Pose2d desiredTargetPose) {
        this.drivetrain = drivetrain;
        this.desiredTargetPose = desiredTargetPose;
        rotationController =
                new PIDController(
                        PhoenixDriveConstants.alignmentkP,
                        PhoenixDriveConstants.alignmentkI,
                        PhoenixDriveConstants.alignmentkD);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        inst = NetworkTableInstance.getDefault();

        this.addRequirements(drivetrain);
    }

    private double getTargetHeading() {
        double targetVectorX = desiredTargetPose.getX() - currentPose.getX();
        double targetVectorY = desiredTargetPose.getY() - currentPose.getY();
        return Math.atan2(targetVectorY, targetVectorX);
    }

    @Override
    public void initialize() {
        // get current pose of robot
        this.currentPose = drivetrain.getState().Pose;
    }

    @Override
    public void execute() {
        this.currentPose = drivetrain.getState().Pose;

        double desiredHeading = this.getTargetHeading();
        double currentHeading = currentPose.getRotation().getRadians();

        double rotationalRate = rotationController.calculate(currentHeading, desiredHeading);

        ChassisSpeeds goalRotationalSpeed = new ChassisSpeeds(0, 0, rotationalRate);

        drivetrain.setGoalSpeeds(goalRotationalSpeed, true);
    }

    @Override
    public boolean isFinished() {
        double desiredHeading = this.getTargetHeading();
        double currentHeading = currentPose.getRotation().getRadians();

        if (Math.abs(desiredHeading - currentHeading)
                < PhoenixDriveConstants.alignToleranceRadians) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        // set drive back to driving mode
    }
}
