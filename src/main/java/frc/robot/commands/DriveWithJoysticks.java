package frc.robot.commands;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.constants.PhoenixDriveConstants;
import frc.robot.subsystems.drive.PhoenixDrive;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveWithJoysticks extends Command {
    PhoenixDrive drivetrain;
    CommandJoystick leftJoystick;
    CommandJoystick rightJoystick;
    Supplier<Vector<N2>> currentVelocitySupplier;

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
            PhoenixDrive drivetrain,
            CommandJoystick leftJoystick,
            CommandJoystick rightJoystick,
            Supplier<Vector<N2>> currentVelocitySupplier
    ) {
        this.drivetrain = drivetrain;
        this.leftJoystick = leftJoystick;
        this.rightJoystick = rightJoystick;
        this.currentVelocitySupplier = currentVelocitySupplier;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        chassisSpeeds = new ChassisSpeeds(leftJoystick.getY(), leftJoystick.getX(), rightJoystick.getX());
        drivetrain.setGoalSpeeds(chassisSpeeds, !rightJoystick.trigger().getAsBoolean());
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
