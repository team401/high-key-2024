package frc.robot.commands;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.constants.PhoenixDriveConstants;
import frc.robot.subsystems.drive.PhoenixDrive;

public class DriveWithJoysticks extends Command {
    PhoenixDrive drivetrain;
    CommandJoystick leftJoystick;
    CommandJoystick rightJoystick;
    NetworkTable driveStats = NetworkTableInstance.getDefault().getTable("Drive");

    double commandedXMpS;
    double commandedYMpS;
    double commandedRotRadpS;

    double lastCommandedXMpS;
    double lastCommandedYMpS;

    double currentXMpS;
    double currentYMpS;

    double lastTime = Utils.getCurrentTimeSeconds();

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

    public DriveWithJoysticks(
            PhoenixDrive drivetrain, CommandJoystick leftJoystick, CommandJoystick rightJoystick) {
        this.drivetrain = drivetrain;
        this.leftJoystick = leftJoystick;
        this.rightJoystick = rightJoystick;

        addRequirements(this.drivetrain);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double filteredVY = -leftJoystick.getY() * PhoenixDriveConstants.maxSpeedMetPerSec;
        double filteredVX = -leftJoystick.getX() * PhoenixDriveConstants.maxSpeedMetPerSec;
        double filteredOmega = rightJoystick.getX() * PhoenixDriveConstants.MaxAngularRateRadPerSec;

        chassisSpeeds = new ChassisSpeeds(filteredVY, filteredVX, filteredOmega);
        drivetrain.setGoalSpeeds(chassisSpeeds, true);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
