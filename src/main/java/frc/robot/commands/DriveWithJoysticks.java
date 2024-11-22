package frc.robot.commands;

import com.ctre.phoenix6.Utils;
import coppercore.math.Deadband;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.constants.ConstantsLoader;
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

    // private Deadband joystickDeadband = new Deadband();

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
        double[] leftJoystickDeadbanded =
                Deadband.twoAxisDeadband(
                        leftJoystick.getX(),
                        leftJoystick.getY(),
                        ConstantsLoader.DriverConstants.joystickDeadband);
        double filteredVY = -leftJoystickDeadbanded[1] * ConstantsLoader.PhoenixDriveConstants.maxSpeedMetPerSec;
        double filteredVX = -leftJoystickDeadbanded[0] * ConstantsLoader.PhoenixDriveConstants.maxSpeedMetPerSec;

        double rightJoystickDeadbanded =
                Deadband.oneAxisDeadband(
                        rightJoystick.getX(), ConstantsLoader.DriverConstants.joystickDeadband);
        double filteredOmega =
                -rightJoystickDeadbanded * ConstantsLoader.PhoenixDriveConstants.MaxAngularRateRadPerSec;

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
