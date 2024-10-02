package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.constants.Constants.AlignTarget;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.PhoenixDriveConstants;
import frc.robot.subsystems.drive.PhoenixDrive;

public class AlignToTarget extends Command {
    private PhoenixDrive drivetrain;
    private CommandJoystick leftJoystick;
    private AlignTarget desiredTarget;
    private Pose2d currentPose;
    private PIDController rotationController;

    public AlignToTarget(
            PhoenixDrive drivetrain, CommandJoystick leftJoystick, AlignTarget desiredTarget) {
        this.drivetrain = drivetrain;
        this.leftJoystick = leftJoystick;
        this.desiredTarget = desiredTarget;

        rotationController =
                new PIDController(
                        PhoenixDriveConstants.alignmentkP,
                        PhoenixDriveConstants.alignmentkI,
                        PhoenixDriveConstants.alignmentkD);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        this.addRequirements(drivetrain);
    }

    private double getTargetHeading(Pose2d desiredTargetPose) {
        double targetVectorX = desiredTargetPose.getX() - currentPose.getX();
        double targetVectorY = desiredTargetPose.getY() - currentPose.getY();
        return Math.atan2(targetVectorY, targetVectorX);
    }

    private double findDesiredHeading() {
        switch (desiredTarget) {
            case SPEAKER:
                drivetrain.setAlignTarget(AlignTarget.SPEAKER);
                if (DriverStation.getAlliance().get() == Alliance.Blue) {
                    return getTargetHeading(
                            new Pose2d(FieldConstants.fieldToBlueSpeaker, new Rotation2d()));
                } else {
                    return getTargetHeading(
                            new Pose2d(FieldConstants.fieldToRedSpeaker, new Rotation2d()));
                }
            case AMP:
                drivetrain.setAlignTarget(AlignTarget.AMP);
                return FieldConstants.ampHeading.getRadians();
            case SOURCE:
                drivetrain.setAlignTarget(AlignTarget.SOURCE);
                if (DriverStation.getAlliance().get() == Alliance.Blue) {
                    return FieldConstants.blueSourceHeading.getRadians();
                } else {
                    return FieldConstants.redSourceHeading.getRadians();
                }
            case UP:
                drivetrain.setAlignTarget(AlignTarget.UP);
                if (DriverStation.getAlliance().get() == Alliance.Blue) {
                    return FieldConstants.blueUpHeading.getRadians();
                } else {
                    return FieldConstants.redUpHeading.getRadians();
                }
            case DOWN:
                drivetrain.setAlignTarget(AlignTarget.DOWN);
                if (DriverStation.getAlliance().get() == Alliance.Blue) {
                    return FieldConstants.blueDownHeading.getRadians();
                } else {
                    return FieldConstants.redDownHeading.getRadians();
                }
            case LEFT:
                drivetrain.setAlignTarget(AlignTarget.LEFT);
                if (DriverStation.getAlliance().get() == Alliance.Blue) {
                    return FieldConstants.blueLeftHeading.getRadians();
                } else {
                    return FieldConstants.redLeftHeading.getRadians();
                }
            case RIGHT:
                drivetrain.setAlignTarget(AlignTarget.RIGHT);
                if (DriverStation.getAlliance().get() == Alliance.Blue) {
                    return FieldConstants.blueRightHeading.getRadians();
                } else {
                    return FieldConstants.redRightHeading.getRadians();
                }
            default:
                // no pose to align to so go back to regular driveWithJoysticks
                drivetrain.setAlignTarget(AlignTarget.NONE);
                this.cancel();
                return 0.0;
        }
    }

    @Override
    public void initialize() {
        // get current pose of robot
        this.currentPose = drivetrain.getState().Pose;
    }

    @Override
    public void execute() {
        this.currentPose = drivetrain.getState().Pose;

        // calculate rotational rate
        double desiredHeading = this.findDesiredHeading();
        double currentHeading = currentPose.getRotation().getRadians();

        double rotationalRate = rotationController.calculate(currentHeading, desiredHeading);

        // add in strafe
        double filteredVY = -leftJoystick.getY() * PhoenixDriveConstants.maxSpeedMetPerSec;
        double filteredVX = -leftJoystick.getX() * PhoenixDriveConstants.maxSpeedMetPerSec;

        ChassisSpeeds goalRotationalSpeed =
                new ChassisSpeeds(filteredVX, filteredVY, rotationalRate);

        drivetrain.setGoalSpeeds(goalRotationalSpeed, true);
    }

    @Override
    public boolean isFinished() {
        double desiredHeading = this.findDesiredHeading();
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
