package frc.robot;

<<<<<<< HEAD
import edu.wpi.first.wpilibj.DriverStation;
=======
import org.littletonrobotics.junction.Logger;

>>>>>>> 44c3b70 (nothing is showing up in advantagekit and i don't know why)
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
<<<<<<< HEAD
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.constants.FeatureFlags;
=======
>>>>>>> 9334e3e (apparently i have to u p d a t e wpilib)
import frc.robot.constants.PhoenixDriveConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drive.PhoenixDrive;
import frc.robot.subsystems.drive.PhoenixDrive.SysIdRoutineType;
import frc.robot.subsystems.drive.commands.DriveWithJoysticks;
<<<<<<< HEAD
import frc.robot.subsystems.localization.CameraContainerReal;
import frc.robot.subsystems.localization.CameraContainerSim;
import frc.robot.subsystems.localization.VisionLocalizer;
=======
import frc.robot.Constants.AlignTarget;
import frc.robot.Constants.ConversionConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.commands.ShootWithGamepad;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeAction;
import frc.robot.subsystems.scoring.AimerIO;
import frc.robot.subsystems.scoring.AimerIOSim;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ShooterIO;
import frc.robot.subsystems.scoring.ShooterIOSim;
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringAction;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
>>>>>>> 44c3b70 (nothing is showing up in advantagekit and i don't know why)

public class RobotContainer {
    PhoenixDrive drive = PhoenixDriveConstants.DriveTrain;
    Telemetry logger = new Telemetry(6);

    //ScoringSubsystem scoringSubsystem;
    ScoringSubsystem scoringSubsystem;
    IntakeSubsystem intakeSubsystem;

    CommandJoystick leftJoystick = new CommandJoystick(0);
    CommandJoystick rightJoystick = new CommandJoystick(1);
<<<<<<< HEAD
    CommandXboxController masher = new CommandXboxController(2);

    VisionLocalizer tagVision;

    public RobotContainer() {
        configureSubsystems();
=======
    CommandXboxController controller = new CommandXboxController(2);

    public RobotContainer() {
        //configureSubsystems();

        scoringSubsystem = new ScoringSubsystem(new ShooterIOSim(), new AimerIOSim());
        intakeSubsystem = new IntakeSubsystem(new IntakeIOSim());

        configureBindings();
        System.out.println("containerrunning");
        SmartDashboard.putString("AlignTarget", "NONE");
    }

    /*public void configureSubsystems() {
        if (true) {
            scoringSubsystem =
                    new ScoringSubsystem(
                            new ShooterIOSim(), new AimerIOSim());
        }

        if (FeatureFlags.runIntake) {
            intakeSubsystem = new IntakeSubsystem(new IntakeIOSim());
        }
    }*/

    // spotless:off
    private void configureBindings() {
        if (true) {
            controller.button(1)
                .onTrue(new InstantCommand(
                        () -> intakeSubsystem.run(IntakeAction.INTAKE)))
                .onFalse(new InstantCommand(
                    () -> intakeSubsystem.run(IntakeAction.NONE)));

            controller.button(2)
                .onTrue(new InstantCommand(
                    () -> intakeSubsystem.run(IntakeAction.REVERSE)))
                .onFalse(new InstantCommand(
                    () -> intakeSubsystem.run(IntakeAction.NONE)));

            // HACK: This button was added during DCMP to un-jam the intake. Ideally, this functionality should be implemented through a state machine.
            controller.button(3)
                .onTrue(new SequentialCommandGroup(new InstantCommand(
                        () -> intakeSubsystem.run(IntakeAction.REVERSE)),
                    Commands.waitSeconds(0.1),
                    new InstantCommand(
                        () -> intakeSubsystem.run(IntakeAction.INTAKE)),
                    Commands.waitSeconds(0.5),
                    new InstantCommand(
                        () -> intakeSubsystem.run(IntakeAction.NONE))))
                .onFalse(new InstantCommand(
                    () -> intakeSubsystem.run(IntakeAction.NONE)));
            
        }

        if (true) {
            
            String alignTarget = "NONE";
            SmartDashboard.getString("AlignTarget", alignTarget);
            System.out.println("aligntargetreading");
            scoringSubsystem.setDefaultCommand(new ShootWithGamepad(
                () -> rightJoystick.getHID().getRawButton(4),
                controller.getHID()::getRightBumper,
                controller.getHID()::getYButton,
                () -> controller.getRightTriggerAxis() > 0.5,
                controller.getHID()::getAButton,
                controller.getHID()::getBButton, scoringSubsystem,
                () -> AlignTarget.valueOf(alignTarget)));
                //FeatureFlags.runDrive ? drivetrain::getAlignTarget : () -> AlignTarget.NONE));

            rightJoystick.button(11).onTrue(new InstantCommand(() -> scoringSubsystem.setArmDisabled(true)));
            rightJoystick.button(16).onTrue(new InstantCommand(() -> scoringSubsystem.setArmDisabled(false)));

            rightJoystick.button(12).onTrue(new InstantCommand(
                () -> {
                    scoringSubsystem.setAction(ScoringAction.OVERRIDE);
                    scoringSubsystem.setVolts(3, 0);
                }, scoringSubsystem));

            rightJoystick.button(15).onTrue(new InstantCommand(
                () -> {
                    scoringSubsystem.setAction(ScoringAction.OVERRIDE);
                    scoringSubsystem.setVolts(-3, 0);
                }, scoringSubsystem));
        }


    } // spotless:on

    public void enabledInit() {

        intakeSubsystem.run(IntakeAction.NONE);
        scoringSubsystem.setAction(ScoringAction.INTAKE);


    }


    /*public void robotPeriodic() {
        if (FeatureFlags.runDrive) {
            Logger.recordOutput(
                    "localizer/whereAmI",
                    FieldFinder.whereAmI(
                            driveTelemetry.getFieldToRobot().getTranslation().getX(),
                            driveTelemetry.getFieldToRobot().getTranslation().getY()));

            Logger.recordOutput("localizer/RobotPose", driveTelemetry.getFieldToRobot());
            Logger.recordOutput(
                    "localizer/RobotVelocity",
                    new Pose2d(
                            driveTelemetry.getFieldToRobot().getX()
                                    + (driveTelemetry.getVelocity().getX()
                                            * DriveConstants.anticipationTime),
                            driveTelemetry.getFieldToRobot().getY()
                                    + (driveTelemetry.getVelocity().getY()
                                            * DriveConstants.anticipationTime),
                            driveTelemetry.getFieldToRobot().getRotation()));

            driveTelemetry.logDataSynchronously();
        }
    }

    public void disabledPeriodic() {
        // set to coast mode when circuit open
        if (brakeSwitch != null && brakeSwitch.get()) {
            if (FeatureFlags.runScoring) {
                scoringSubsystem.setBrakeMode(false);
            }
            if (FeatureFlags.runEndgame) {
                endgameSubsystem.setBrakeMode(false);
            }
            if (FeatureFlags.runDrive) {
                drivetrain.setBrakeMode(false);
            }
        } else {
            if (FeatureFlags.runScoring) {
                scoringSubsystem.setBrakeMode(true);
            }
            if (FeatureFlags.runEndgame) {
                endgameSubsystem.setBrakeMode(true);
            }
            if (FeatureFlags.runDrive) {
                drivetrain.setBrakeMode(true);
            }
        }
        if (ledSwitch != null && leds != null) {
            leds.setEnabled(!ledSwitch.get());
        }
    }

    public void autoInit() {
        if (drivetrain.getAutoCommand() != null) {
            drivetrain.autoInit();

            drivetrain.getAutoCommand().schedule();

            if (FeatureFlags.runScoring) {
                scoringSubsystem.setAction(ScoringSubsystem.ScoringAction.SHOOT);
            }
            if (FeatureFlags.runIntake) {
                intakeSubsystem.run(IntakeAction.INTAKE);
            }
        }
    }

    private void configureAutonomous() {
        if (FeatureFlags.runScoring) {
            NamedCommands.registerCommand(
                    "Shoot Scoring",
                    new InstantCommand(
                            () -> {
                                if (FeatureFlags.runScoring) {
                                    scoringSubsystem.setAction(
                                            ScoringSubsystem.ScoringAction.SHOOT);
                                }
                                if (FeatureFlags.runIntake) {
                                    intakeSubsystem.run(IntakeAction.INTAKE);
                                }
                            }));
            NamedCommands.registerCommand(
                    "Aim Scoring",
                    new InstantCommand(
                            () -> scoringSubsystem.setAction(ScoringSubsystem.ScoringAction.AIM)));
            NamedCommands.registerCommand(
                    "Intake Scoring",
                    new InstantCommand(
                            () ->
                                    scoringSubsystem.setAction(
                                            ScoringSubsystem.ScoringAction.INTAKE)));
            NamedCommands.registerCommand(
                    "Wait Scoring",
                    new InstantCommand(
                            () -> scoringSubsystem.setAction(ScoringSubsystem.ScoringAction.WAIT)));
            NamedCommands.registerCommand(
                    "OverrideStageAvoidance",
                    new InstantCommand(() -> scoringSubsystem.setOverrideStageAvoidance(true)));
            NamedCommands.registerCommand(
                    "Un-OverrideStageAvoidance",
                    new InstantCommand(() -> scoringSubsystem.setOverrideStageAvoidance(false)));
        } else {
            NamedCommands.registerCommand("Shoot Scoring", Commands.none());
            NamedCommands.registerCommand("Aim Scoring", Commands.none());
            NamedCommands.registerCommand("Wait Scoring", Commands.none());
            NamedCommands.registerCommand("Intake Scoring", Commands.none());
            NamedCommands.registerCommand("OverrideStageAvoidance", Commands.none());
            NamedCommands.registerCommand("Un-OverrideStageAvoidance", Commands.none());
        }
        if (FeatureFlags.runIntake) {
            NamedCommands.registerCommand("Intake Note", Commands.none());
        } else {
            NamedCommands.registerCommand("Intake Note", Commands.none());
        }
        if (FeatureFlags.runDrive) {
            NamedCommands.registerCommand(
                    "Disable Auto-Align",
                    new InstantCommand(() -> drivetrain.setAlignState(AlignState.MANUAL)));
            NamedCommands.registerCommand(
                    "Enable Auto-Align",
                    new InstantCommand(() -> drivetrain.setAlignState(AlignState.ALIGNING)));
        } else {
            NamedCommands.registerCommand("Disable Auto-Align", Commands.none());
            NamedCommands.registerCommand("Enable Auto-Align", Commands.none());
        }
        if (FeatureFlags.runScoring) {
            NamedCommands.registerCommand(
                    "Override Shoot",
                    new InstantCommand(() -> scoringSubsystem.setOverrideShoot(true)));
            NamedCommands.registerCommand(
                    "Un-Override Shoot",
                    new InstantCommand(() -> scoringSubsystem.setOverrideShoot(false)));
        } else {
            NamedCommands.registerCommand("Override Shoot", Commands.none());
            NamedCommands.registerCommand("Un-Override Shoot", Commands.none());
        }
    }

    public void teleopInit() {
>>>>>>> e34a9db (wip)
>>>>>>> 9334e3e (apparently i have to u p d a t e wpilib)
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

        // This is in teleopInit to prevent it from wasting time in auto
        if (FeatureFlags.runScoring) {
            // scoringSubsystem.homeHood();

            scoringSubsystem.setAction(ScoringAction.WAIT);
            scoringSubsystem.enabledInit();
        }

        if (FeatureFlags.runDrive) {
            drivetrain.teleopInit();
        }

        SmartDashboard.putNumber("Debug/currentTimeMillis", System.currentTimeMillis());
    }*/
}
