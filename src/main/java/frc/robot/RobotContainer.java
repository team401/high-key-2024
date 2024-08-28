package frc.robot;

<<<<<<< HEAD
import edu.wpi.first.wpilibj.DriverStation;
=======
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
>>>>>>> 7c58bf4 (build.gradle and org.littleton are not working, unsure why)
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.constants.FeatureFlags;
import frc.robot.constants.PhoenixDriveConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drive.PhoenixDrive;
import frc.robot.subsystems.drive.PhoenixDrive.SysIdRoutineType;
import frc.robot.subsystems.drive.commands.DriveWithJoysticks;
import frc.robot.subsystems.localization.CameraContainerReal;
import frc.robot.subsystems.localization.CameraContainerSim;
import frc.robot.subsystems.localization.VisionLocalizer;

public class RobotContainer {
    PhoenixDrive drive = PhoenixDriveConstants.DriveTrain;
    Telemetry logger = new Telemetry(6);
    CommandJoystick leftJoystick = new CommandJoystick(0);
    CommandJoystick rightJoystick = new CommandJoystick(1);
    CommandXboxController masher = new CommandXboxController(2);

    VisionLocalizer tagVision;

    public RobotContainer() {
        configureSubsystems();
        configureBindings();
    }

    private void configureBindings() {
        drive.registerTelemetry(logger::telemeterize);
        drive.setDefaultCommand(new DriveWithJoysticks(drive, leftJoystick, rightJoystick));

        if (DriverStation.isTest()) {
            // SYS ID

            /* Bindings for switching routines */
            /* DPad up = Translation; DPad down = Rotation; DPad right = Steer */

            masher.povDown()
                    .onTrue(
                            Commands.runOnce(
                                    () -> drive.setSysIdRoutine(SysIdRoutineType.Rotation), drive));
            masher.povRight()
                    .onTrue(
                            Commands.runOnce(
                                    () -> drive.setSysIdRoutine(SysIdRoutineType.Steer), drive));
            masher.povUp()
                    .onTrue(
                            Commands.runOnce(
                                    () -> drive.setSysIdRoutine(SysIdRoutineType.Translation),
                                    drive));

            /* Bindings for drivetrain characterization */
            /* These bindings require multiple buttons pushed to swap between quastatic and dynamic */
            /* Back/Start select dynamic/quasistatic, Y/X select forward/reverse direction */

            masher.back().and(masher.y()).whileTrue(drive.sysIdDynamic(Direction.kForward));
            masher.back().and(masher.x()).whileTrue(drive.sysIdDynamic(Direction.kReverse));
            masher.start().and(masher.y()).whileTrue(drive.sysIdQuasistatic(Direction.kForward));
            masher.start().and(masher.x()).whileTrue(drive.sysIdQuasistatic(Direction.kReverse));
        }
    }

    private void configureSubsystems() {
        // TODO: Potentially migrate to Constants.mode
        if (Robot.isReal()) {
            if (FeatureFlags.runVision) {
                tagVision = new VisionLocalizer(new CameraContainerReal(VisionConstants.cameras));
            }
        } else {
            if (FeatureFlags.simulateVision) {
                tagVision =
                        new VisionLocalizer(
                                new CameraContainerSim(
                                        VisionConstants.cameras, logger::getModuleStates));
            }
        }

        if (FeatureFlags.runVision && Robot.isReal()
                || FeatureFlags.simulateVision && !Robot.isReal()) {
            tagVision.setCameraConsumer(
                    (m) -> drive.addVisionMeasurement(m.pose(), m.timestamp(), m.variance()));
        }
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
