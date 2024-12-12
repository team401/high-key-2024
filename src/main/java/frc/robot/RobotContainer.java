package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import coppercore.wpilib_interface.DriveWithJoysticks;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
// import frc.robot.commands.ShootWithGamepad;
import frc.robot.constants.DriverConstants;
import frc.robot.constants.FeatureFlags;
import frc.robot.constants.ModeConstants;
import frc.robot.constants.PhoenixDriveConstants;
import frc.robot.constants.PhoenixDriveConstants.AlignTarget;
import frc.robot.constants.ScoringConstants;
import frc.robot.constants.VisionConstants;
// import frc.robot.subsystems.LED;
// import frc.robot.subsystems.OrchestraSubsystem;
import frc.robot.subsystems.drive.PhoenixDrive;
import frc.robot.subsystems.drive.PhoenixDrive.SysIdRoutineType;
// import frc.robot.subsystems.intake.IntakeIO;
// import frc.robot.subsystems.intake.IntakeIOSim;
// import frc.robot.subsystems.intake.IntakeNEOVortex;
// import frc.robot.subsystems.intake.IntakeSubsystem;
// import frc.robot.subsystems.intake.IntakeSubsystem.IntakeAction;
// import frc.robot.subsystems.localization.CameraContainerReal;
// import frc.robot.subsystems.localization.CameraContainerReplay;
// import frc.robot.subsystems.localization.CameraContainerSim;
// import frc.robot.subsystems.localization.VisionLocalizer;
// import frc.robot.subsystems.scoring.AimerIO;
// import frc.robot.subsystems.scoring.AimerIORoboRio;
// import frc.robot.subsystems.scoring.AimerIOSim;
// import frc.robot.subsystems.scoring.ScoringSubsystem;
// import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringAction;
// import frc.robot.subsystems.scoring.ShooterIO;
// import frc.robot.subsystems.scoring.ShooterIOSim;
// import frc.robot.subsystems.scoring.ShooterIOTalonFX;
import frc.robot.utils.AllianceUtil;
// import frc.robot.utils.feedforward.TuneG;
// import frc.robot.utils.feedforward.TuneS;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import coppercore.vision.VisionIOPhotonReal;
import coppercore.vision.VisionLocalizer;

public class RobotContainer {
    PhoenixDrive drive;

    Telemetry logger;

//     ScoringSubsystem scoringSubsystem;
//     IntakeSubsystem intakeSubsystem;
//     LED leds;

//     OrchestraSubsystem orchestraSubsystem;

    CommandJoystick leftJoystick = new CommandJoystick(0);
    CommandJoystick rightJoystick = new CommandJoystick(1);
    CommandXboxController masher = new CommandXboxController(2);

    VisionLocalizer tagVision;

    SendableChooser<String> testModeChooser = new SendableChooser<String>();

    public RobotContainer() {
        configureSubsystems();
        configureSuppliers();
        configureNamedCommands();
        configureBindings();
        configureModes();
    }

    private void configureNamedCommands() {
        NamedCommands.registerCommand(
                "alignToSpeaker",
                new InstantCommand(
                        () -> {
                            drive.setAlignTarget(AlignTarget.SPEAKER);
                            drive.setAligning(true);
                        }));

        NamedCommands.registerCommand(
                "stopAlignToSpeaker", new InstantCommand(() -> drive.setAligning(false)));

        // NamedCommands.registerCommand(
        //         "intakeNote",
        //         // intakes to start, ends by setting action to NONE when intake subsystem has note
        //         new InstantCommand(
        //                 () -> {
        //                     intakeSubsystem.run(IntakeAction.INTAKE);
        //                     scoringSubsystem.setAction(ScoringAction.INTAKE);
        //                 },
        //                 intakeSubsystem,
        //                 scoringSubsystem));

        // NamedCommands.registerCommand(
        //         "waitForAlignment",
        //         Commands.waitUntil(() -> drive.isDriveAligned() && !scoringSubsystem.hasNote()));

        // NamedCommands.registerCommand(
        //         "shootNoteAtSpeaker",
        //         new InstantCommand(() -> scoringSubsystem.setAction(ScoringAction.SHOOT)));
    }

    private void configureSubsystems() {
        if (FeatureFlags.runDrive) {
            initDrive();
        }
        if (FeatureFlags.runVision) {
            initVision();
        }
        // if (FeatureFlags.runScoring) {
        //     initScoring();
        // }
        // if (FeatureFlags.runIntake) {
        //     initIntake();
        // }
        // if (FeatureFlags.runScoring && FeatureFlags.runIntake && FeatureFlags.runLEDS) {
        //     initLEDs();
        // }
        // if (FeatureFlags.runOrchestra) {
        //     initOrchestra();
        // }
    }

    private void initDrive() {
        switch (ModeConstants.currentMode) {
            case REAL:
                drive =
                        new PhoenixDrive(
                                PhoenixDriveConstants.DrivetrainConstants,
                                PhoenixDriveConstants.FrontLeft,
                                PhoenixDriveConstants.FrontRight,
                                PhoenixDriveConstants.BackLeft,
                                PhoenixDriveConstants.BackRight);
                logger = new Telemetry(6);
                break;
            case SIM:
                drive =
                        new PhoenixDrive(
                                PhoenixDriveConstants.DrivetrainConstants,
                                PhoenixDriveConstants.FrontLeft,
                                PhoenixDriveConstants.FrontRight,
                                PhoenixDriveConstants.BackLeft,
                                PhoenixDriveConstants.BackRight);

                logger = new Telemetry(6);
                break;
            case REPLAY:
                drive =
                        new PhoenixDrive(
                                PhoenixDriveConstants.DrivetrainConstants,
                                PhoenixDriveConstants.FrontLeft,
                                PhoenixDriveConstants.FrontRight,
                                PhoenixDriveConstants.BackLeft,
                                PhoenixDriveConstants.BackRight);

                logger = new Telemetry(6);
                break;
        }
    }

    private void initVision() {
        switch (ModeConstants.currentMode) {
            case REAL:
                tagVision = new VisionLocalizer(drive::addPhotonVisionMeasurement, VisionConstants.fieldLayout, new double[4], new VisionIOPhotonReal("front", new Transform3d()));
                break;
        //     case SIM:
        //         if (FeatureFlags.runDrive) {
        //             tagVision =
        //                     new VisionLocalizer(
        //                             new CameraContainerSim(
        //                                     VisionConstants.cameras, logger::getModuleStates));
        //         } else {
        //             /* TODO: Maybe try to spoof vision sim without drive telemetry by giving it
        //             all zeros */
        //             throw new NullPointerException("Vision simulation depends on drive!");
        //         }
        //         break;
        //     case REPLAY:
        //         tagVision = new VisionLocalizer(new CameraContainerReplay(VisionConstants.cameras));
        //         break;
        }
    }

//     private void initIntake() {
//         switch (ModeConstants.currentMode) {
//             case REAL:
//                 intakeSubsystem = new IntakeSubsystem(new IntakeNEOVortex());
//                 break;
//             case SIM:
//                 intakeSubsystem = new IntakeSubsystem(new IntakeIOSim());
//                 break;
//             case REPLAY:
//                 intakeSubsystem = new IntakeSubsystem(new IntakeIO() {});
//                 break;
//         }
//     }

//     private void initScoring() {
//         switch (ModeConstants.currentMode) {
//             case REAL:
//                 scoringSubsystem =
//                         new ScoringSubsystem(new ShooterIOTalonFX(), new AimerIORoboRio());
//                 break;
//             case SIM:
//                 scoringSubsystem = new ScoringSubsystem(new ShooterIOSim(), new AimerIOSim());
//                 break;
//             case REPLAY:
//                 scoringSubsystem = new ScoringSubsystem(new ShooterIO() {}, new AimerIO() {});
//                 break;
//         }
//     }

//     private void initLEDs() {
//         leds = new LED(scoringSubsystem, intakeSubsystem);
//     }

//     private void initOrchestra() {
//         switch (ModeConstants.currentMode) {
//             case REAL:
//                 orchestraSubsystem =
//                         new OrchestraSubsystem(
//                                 "music/mii_channel.chrp"); // TODO: Add music files to deploy!
//                 if (FeatureFlags.runScoring) {
//                     orchestraSubsystem.addInstruments(
//                             scoringSubsystem.getAimerIO().getOrchestraMotors());
//                     orchestraSubsystem.addInstruments(
//                             scoringSubsystem.getShooterIO().getOrchestraMotors());
//                 }

//                 if (FeatureFlags.runDrive) {
//                     orchestraSubsystem.addInstrument(drive.getModule(0).getDriveMotor());
//                     orchestraSubsystem.addInstrument(drive.getModule(0).getSteerMotor());
//                     orchestraSubsystem.addInstrument(drive.getModule(1).getDriveMotor());
//                     orchestraSubsystem.addInstrument(drive.getModule(1).getSteerMotor());
//                     orchestraSubsystem.addInstrument(drive.getModule(2).getDriveMotor());
//                     orchestraSubsystem.addInstrument(drive.getModule(2).getSteerMotor());
//                     orchestraSubsystem.addInstrument(drive.getModule(3).getDriveMotor());
//                     orchestraSubsystem.addInstrument(drive.getModule(3).getSteerMotor());
//                 }
//             default:
//                 break;
//         }
//     }

    private void configureSuppliers() {
        if (FeatureFlags.runScoring) {
            Supplier<Pose2d> poseSupplier;
            if (drive != null) {
                poseSupplier = () -> drive.getState().Pose;
            } else {
                poseSupplier = () -> new Pose2d();
            }

        //     scoringSubsystem.setPoseSupplier(poseSupplier);
        //     scoringSubsystem.setDriveAlignedSupplier(() -> drive.isDriveAligned());
        }

        if (FeatureFlags.runIntake) {
            BooleanSupplier shooterHasNote;
            BooleanSupplier shooterInIntakePosition;
        //     if (scoringSubsystem != null) {
        //         shooterHasNote =
        //                 () -> {
        //                     return scoringSubsystem.hasNote();
        //                 };
        //         shooterInIntakePosition =
        //                 () -> {
        //                     return scoringSubsystem.aimerAtIntakePosition();
        //                 };
        //     } else {
        //         shooterHasNote = () -> false;
        //         shooterInIntakePosition = () -> false;
        //     }
        //     intakeSubsystem.setShooterHasNoteSupplier(shooterHasNote);
        //     intakeSubsystem.setShooterAtIntakePosition(shooterInIntakePosition);
        }

        // if (FeatureFlags.runVision) {
        //     if (drive != null) {
        //         tagVision.setCameraConsumer(
        //                 (m) -> drive.addVisionMeasurement(m.pose(), m.timestamp(), m.variance()));
        //     } else {
        //         tagVision.setCameraConsumer((m) -> {});
        //     }
        // }

        if (FeatureFlags.runLEDS) {
            if (FeatureFlags.runVision) {
                // leds.setVisionWorkingSupplier(() -> tagVision.coprocessorConnected());
            } else {
                // leds.setVisionWorkingSupplier(() -> false);
            }
        }
    }

    private void configureBindings() {
        if (drive != null) {
            drive.registerTelemetry(logger::telemeterize);
            setUpDriveWithJoysticks();
        }
        if (DriverStation.isTest()) {
            // SYS ID
            if (drive != null) {
                // NOTE: These if-statements are nested because other subsystems will go here too

                /* Bindings for switching routines */
                // /* DPad up = Translation; DPad down = Rotation; DPad right = Steer */

                // masher.povDown()
                //         .onTrue(
                //                 Commands.runOnce(
                //                         () -> drive.setSysIdRoutine(SysIdRoutineType.Rotation),
                //                         drive));
                // masher.povRight()
                //         .onTrue(
                //                 Commands.runOnce(
                //                         () -> drive.setSysIdRoutine(SysIdRoutineType.Steer),
                //                         drive));
                // masher.povUp()
                //         .onTrue(
                //                 Commands.runOnce(
                //                         () -> drive.setSysIdRoutine(SysIdRoutineType.Translation),
                //                         drive));

                /* Bindings for drivetrain characterization */
                /*
                 * These bindings require multiple buttons pushed to swap between quastatic and
                 * dynamic
                 */
                /*
                 * Back/Start select dynamic/quasistatic, Y/X select forward/reverse direction
                 */

                // masher.back().and(masher.y()).whileTrue(drive.sysIdDynamic(Direction.kForward));
                // masher.back().and(masher.x()).whileTrue(drive.sysIdDynamic(Direction.kReverse));
                // masher.start()
                //         .and(masher.y())
                //         .whileTrue(drive.sysIdQuasistatic(Direction.kForward));
                // masher.start()
                //         .and(masher.x())
                //         .whileTrue(drive.sysIdQuasistatic(Direction.kReverse));
            }
        }

        if (FeatureFlags.runIntake) {
        //     masher.b()
        //             .onTrue(new InstantCommand(() -> intakeSubsystem.run(IntakeAction.INTAKE)))
        //             .onFalse(new InstantCommand(() -> intakeSubsystem.run(IntakeAction.NONE)));

        //     masher.a()
        //             .onTrue(new InstantCommand(() -> intakeSubsystem.run(IntakeAction.REVERSE)))
        //             .onFalse(new InstantCommand(() -> intakeSubsystem.run(IntakeAction.NONE)));

            // HACK: This button was added during DCMP to un-jam the intake. Ideally, this
            // functionality should be implemented through a state machine.
        //     masher.x()
        //             .onTrue(
        //                     new SequentialCommandGroup(
        //                             new InstantCommand(
        //                                     () -> intakeSubsystem.run(IntakeAction.REVERSE)),
        //                             Commands.waitSeconds(0.1),
        //                             new InstantCommand(
        //                                     () -> intakeSubsystem.run(IntakeAction.INTAKE)),
        //                             Commands.waitSeconds(0.5),
        //                             new InstantCommand(
        //                                     () -> intakeSubsystem.run(IntakeAction.NONE))))
        //             .onFalse(new InstantCommand(() -> intakeSubsystem.run(IntakeAction.NONE)));
        }

        // if (FeatureFlags.runScoring) {
        //     scoringSubsystem.setDefaultCommand(
        //             new ShootWithGamepad(
        //                     () -> rightJoystick.getHID().getRawButton(4),
        //                     masher.getHID()::getRightBumper,
        //                     masher.getHID()::getYButton,
        //                     () -> masher.getRightTriggerAxis() > 0.5,
        //                     masher.getHID()::getAButton,
        //                     masher.getHID()::getBButton,
        //                     scoringSubsystem,
        //                     () -> drive.getAlignTarget()));
        //     // FeatureFlags.runDrive ? drivetrain::getAlignTarget : () -> AlignTarget.NONE));

        //     rightJoystick
        //             .button(11)
        //             .onTrue(new InstantCommand(() -> scoringSubsystem.setArmDisabled(true)));
        //     rightJoystick
        //             .button(16)
        //             .onTrue(new InstantCommand(() -> scoringSubsystem.setArmDisabled(false)));

        //     rightJoystick
        //             .button(12)
        //             .onTrue(
        //                     new InstantCommand(
        //                             () -> {
        //                                 scoringSubsystem.setAction(ScoringAction.OVERRIDE);
        //                                 scoringSubsystem.setVolts(3, 0);
        //                             },
        //                             scoringSubsystem));

        //     rightJoystick
        //             .button(15)
        //             .onTrue(
        //                     new InstantCommand(
        //                             () -> {
        //                                 scoringSubsystem.setAction(ScoringAction.OVERRIDE);
        //                                 scoringSubsystem.setVolts(-3, 0);
        //                             },
        //                             scoringSubsystem));

        //     masher.povUp();
        // }
        if (FeatureFlags.runDrive) {
            masher.povUp()
                    .onTrue(new InstantCommand(() -> drive.setAlignTarget(AlignTarget.SPEAKER)));

            masher.povRight()
                    .onTrue(new InstantCommand(() -> drive.setAlignTarget(AlignTarget.AMP)));

            masher.povLeft()
                    .onTrue(new InstantCommand(() -> drive.setAlignTarget(AlignTarget.SOURCE)));

            masher.povDown()
                    .onTrue(new InstantCommand(() -> drive.setAlignTarget(AlignTarget.PASS)));

            leftJoystick
                    .trigger()
                    .onTrue(new InstantCommand(() -> drive.setAligning(true)))
                    .onFalse(new InstantCommand(() -> drive.setAligning(false)));

        //     leftJoystick
        //             .top()
        //             .onTrue(
        //                     new InstantCommand(
        //                             () ->
        //                                     drive.seedFieldRelative(
        //                                             AllianceUtil.getPoseAgainstSpeaker())));

            rightJoystick
                    .povUp()
                    .onTrue(new InstantCommand(() -> drive.setAlignTarget(AlignTarget.SPEAKER)));

            rightJoystick
                    .povDown()
                    .onTrue(new InstantCommand(() -> drive.setAlignTarget(AlignTarget.AMP)));

            rightJoystick
                    .povLeft()
                    .onTrue(new InstantCommand(() -> drive.setAlignTarget(AlignTarget.SOURCE)));

            rightJoystick
                    .povRight()
                    .onTrue(new InstantCommand(() -> drive.setAlignTarget(AlignTarget.PASS)));
        }
    } // spotless:on

    public void enabledInit() {
        // intakeSubsystem.run(IntakeAction.NONE);
        // scoringSubsystem.setAction(ScoringAction.WAIT);
    }

    public void onDSConnect() {
        // drive.configurePathPlanner();
    }

    public Command getAutonomousCommand() {
        return drive.getAutoPath();
    }

    private void configureModes() {
        testModeChooser.setDefaultOption("Blank", "tuning");

        testModeChooser.addOption("Shooter Tuning", "tuning-shooter");
        testModeChooser.addOption("Aimer Tuning", "tuning-aimer");
        testModeChooser.addOption("Shot Tuning", "tuning-shot");
        testModeChooser.addOption("Amp Tuning", "tuning-amp");

        SmartDashboard.putData("Test Mode Chooser", testModeChooser);
    }

    public void teleopInit() {
        if (drive != null) {
            drive.setAligning(false);
        }
    }

    public void testInit() {
        // Reset bindings
        masher = new CommandXboxController(2);

        switch (testModeChooser.getSelected()) {
            case "tuning":
                break;
            case "tuning-shooter":
                SmartDashboard.putNumber("Test-Mode/shooter/kP", ScoringConstants.shooterkP);
                SmartDashboard.putNumber("Test-Mode/shooter/kI", ScoringConstants.shooterkI);
                SmartDashboard.putNumber("Test-Mode/shooter/kD", ScoringConstants.shooterkD);

                SmartDashboard.putNumber("Test-Mode/shooter/setpointPosition", 0.25);
                SmartDashboard.putNumber("Test-Mode/shooter/volts", 2.0);

                // scoringSubsystem.setAction(ScoringAction.OVERRIDE);

                // // TODO: Add Tunables to coppercore!
                // masher.a().onTrue(new TuneS(scoringSubsystem, 1));

                // masher.b().onTrue(new TuneG(scoringSubsystem, 1));

                // masher.y()
                //         .onTrue(
                //                 new InstantCommand(
                //                         () ->
                //                                 scoringSubsystem.setPID(
                //                                         SmartDashboard.getNumber(
                //                                                 "Test-Mode/shooter/kP",
                //                                                 ScoringConstants.shooterkP),
                //                                         SmartDashboard.getNumber(
                //                                                 "Test-Mode/shooter/kI",
                //                                                 ScoringConstants.shooterkI),
                //                                         SmartDashboard.getNumber(
                //                                                 "Test-Mode/shooter/kD",
                //                                                 ScoringConstants.shooterkD),
                //                                         1)))
                //         .onTrue(
                //                 new InstantCommand(
                //                         () ->
                //                                 scoringSubsystem.runToPosition(
                //                                         SmartDashboard.getNumber(
                //                                                 "Test-Mode/shooter/setpointPosition",
                //                                                 0.25),
                //                                         1)))
                //         .onTrue(
                //                 new InstantCommand(
                //                         () ->
                //                                 scoringSubsystem.setAction(
                //                                         ScoringAction.TEMPORARY_SETPOINT)))
                //         .onFalse(
                //                 new InstantCommand(
                //                         () -> scoringSubsystem.setAction(ScoringAction.OVERRIDE)));
                break;
            case "tuning-shot":
                // scoringSubsystem.setAction(ScoringAction.TUNING);
                SmartDashboard.putNumber("Test-Mode/aimer/setpointPosition", 0.0);
                SmartDashboard.putNumber("Test-Mode/shooter/setpointRPM", 2000);

                setUpDriveWithJoysticks();

                // masher.y()
                //         .onTrue(
                //                 new InstantCommand(
                //                         () ->
                //                                 scoringSubsystem.runToPosition(
                //                                         SmartDashboard.getNumber(
                //                                                 "Test-Mode/aimer/setpointPosition",
                //                                                 0.0),
                //                                         0)))
                //         .onTrue(
                //                 new InstantCommand(
                //                         () ->
                //                                 scoringSubsystem.runToPosition(
                //                                         SmartDashboard.getNumber(
                //                                                 "Test-Mode/shooter/setpointRPM",
                //                                                 2000),
                //                                         2)))
                //         .onTrue(
                //                 new InstantCommand(
                //                         () ->
                //                                 scoringSubsystem.setAction(
                //                                         ScoringAction.TEMPORARY_SETPOINT)))
                //         .onFalse(
                //                 new InstantCommand(
                //                         () -> scoringSubsystem.setAction(ScoringAction.OVERRIDE)));

                // masher.leftBumper()
                //         .onTrue(new InstantCommand(() -> scoringSubsystem.setTuningKickerVolts(12)))
                //         .onFalse(
                //                 new InstantCommand(() -> scoringSubsystem.setTuningKickerVolts(0)));
                break;
            case "tuning-aimer":
                SmartDashboard.putNumber("Test-Mode/aimer/kP", ScoringConstants.aimerkP);
                SmartDashboard.putNumber("Test-Mode/aimer/kI", ScoringConstants.aimerkI);
                SmartDashboard.putNumber("Test-Mode/aimer/kD", ScoringConstants.aimerkD);

                SmartDashboard.putNumber("Test-Mode/aimer/kG", ScoringConstants.aimerkG);
                SmartDashboard.putNumber("Test-Mode/aimer/kS", ScoringConstants.aimerkS);

                SmartDashboard.putNumber(
                        "Test-Mode/aimer/profileMaxVelocity", ScoringConstants.aimerCruiseVelocity);
                SmartDashboard.putNumber(
                        "Test-Mode/aimer/profileMaxAcceleration",
                        ScoringConstants.aimerAcceleration);

                SmartDashboard.putNumber("Test-Mode/aimer/setpointPosition", 0.25);
                SmartDashboard.putNumber("Test-Mode/aimer/volts", 2.0);

                // scoringSubsystem.setAction(ScoringAction.OVERRIDE);

                // masher.a().onTrue(new TuneS(scoringSubsystem, 0));

                // masher.b().onTrue(new TuneG(scoringSubsystem, 0));

                // masher.y()
                //         .onTrue(
                //                 new InstantCommand(
                //                         () ->
                //                                 scoringSubsystem.setPID(
                //                                         SmartDashboard.getNumber(
                //                                                 "Test-Mode/aimer/kP",
                //                                                 ScoringConstants.aimerkP),
                //                                         SmartDashboard.getNumber(
                //                                                 "Test-Mode/aimer/kI",
                //                                                 ScoringConstants.aimerkI),
                //                                         SmartDashboard.getNumber(
                //                                                 "Test-Mode/aimer/kD",
                //                                                 ScoringConstants.aimerkD),
                //                                         0)))
                //         .onTrue(
                //                 new InstantCommand(
                //                         () ->
                //                                 scoringSubsystem.setMaxProfileProperties(
                //                                         SmartDashboard.getNumber(
                //                                                 "Test-Mode/aimer/profileMaxVelocity",
                //                                                 ScoringConstants
                //                                                         .aimerCruiseVelocity),
                //                                         SmartDashboard.getNumber(
                //                                                 "Test-Mode/aimer/profileMaxAcceleration",
                //                                                 ScoringConstants.aimerAcceleration),
                //                                         0)))
                //         .onTrue(
                //                 new InstantCommand(
                //                         () ->
                //                                 scoringSubsystem.setFF(
                //                                         SmartDashboard.getNumber(
                //                                                 "Test-Mode/aimer/kS",
                //                                                 ScoringConstants.aimerkS),
                //                                         0.0,
                //                                         0.0,
                //                                         SmartDashboard.getNumber(
                //                                                 "Test-Mode/aimer/kG",
                //                                                 ScoringConstants.aimerkG),
                //                                         0)))
                //         .onTrue(
                //                 new InstantCommand(
                //                         () ->
                //                                 scoringSubsystem.runToPosition(
                //                                         SmartDashboard.getNumber(
                //                                                 "Test-Mode/aimer/setpointPosition",
                //                                                 0.0),
                //                                         0)))
                //         .onTrue(
                //                 new InstantCommand(
                //                         () ->
                //                                 scoringSubsystem.setAction(
                //                                         ScoringAction.TEMPORARY_SETPOINT)))
                //         .onFalse(
                //                 new InstantCommand(
                //                         () -> scoringSubsystem.setAction(ScoringAction.OVERRIDE)));

                // masher.povUp()
                //         .onTrue(new InstantCommand(() -> scoringSubsystem.runToPosition(1.1, 0)))
                //         .onTrue(
                //                 new InstantCommand(
                //                         () ->
                //                                 scoringSubsystem.setAction(
                //                                         ScoringAction.TEMPORARY_SETPOINT)))
                //         .onFalse(
                //                 new InstantCommand(
                //                         () -> scoringSubsystem.setAction(ScoringAction.OVERRIDE)));

                // masher.povDown()
                //         .onTrue(new InstantCommand(() -> scoringSubsystem.runToPosition(0.0, 0)))
                //         .onTrue(
                //                 new InstantCommand(
                //                         () ->
                //                                 scoringSubsystem.setAction(
                //                                         ScoringAction.TEMPORARY_SETPOINT)))
                //         .onFalse(
                //                 new InstantCommand(
                //                         () -> scoringSubsystem.setAction(ScoringAction.OVERRIDE)));

                // masher.leftBumper()
                //         .onTrue(
                //                 new InstantCommand(
                //                         () ->
                //                                 scoringSubsystem.setVolts(
                //                                         SmartDashboard.getNumber(
                //                                                 "Test-Mode/aimer/volts", 2.0),
                //                                         0)))
                //         .onFalse(new InstantCommand(() -> scoringSubsystem.setVolts(0, 0)));

                // masher.rightBumper()
                //         .onTrue(
                //                 new InstantCommand(
                //                         () ->
                //                                 scoringSubsystem.setVolts(
                //                                         -SmartDashboard.getNumber(
                //                                                 "Test-Mode/aimer/volts", 2.0),
                //                                         0)))
                //         .onFalse(new InstantCommand(() -> scoringSubsystem.setVolts(0, 0)));
                break;
            case "tuning-amp":
                SmartDashboard.putNumber(
                        "Test-Mode/amp/aimerSetpointPosition",
                        ScoringConstants.ampAimerAngleRotations);

                // Let us drive
                CommandScheduler.getInstance().cancelAll();
                setUpDriveWithJoysticks();

                // Reset bindings
                masher = new CommandXboxController(2);

                // Let us intake
                // masher.b()
                //         .onTrue(new InstantCommand(() -> intakeSubsystem.run(IntakeAction.INTAKE)))
                //         .onFalse(new InstantCommand(() -> intakeSubsystem.run(IntakeAction.NONE)));

                // // Let us reverse intake (in case a note gets jammed during amp tuning)
                // masher.a()
                //         .onTrue(new InstantCommand(() -> intakeSubsystem.run(IntakeAction.REVERSE)))
                //         .onFalse(new InstantCommand(() -> intakeSubsystem.run(IntakeAction.NONE)));

                // masher.rightTrigger()
                //         .onTrue(
                //                 new InstantCommand(
                //                         () ->
                //                                 scoringSubsystem.runToPosition(
                //                                         SmartDashboard.getNumber(
                //                                                 "Test-Mode/amp/aimerSetpointPosition",
                //                                                 0.0),
                //                                         0)))
                //         .onTrue(
                //                 new InstantCommand(
                //                         () ->
                //                                 scoringSubsystem.setAction(
                //                                         ScoringAction.TEMPORARY_SETPOINT)))
                //         .onFalse(
                //                 new InstantCommand(
                //                         () -> scoringSubsystem.setAction(ScoringAction.OVERRIDE)));

                // masher.rightBumper()
                //         .onTrue(
                //                 new InstantCommand(
                //                         () ->
                //                                 scoringSubsystem.setOverrideKickerVoltsDirectly(
                //                                         12.0)))
                //         .onFalse(
                //                 new InstantCommand(
                //                         () ->
                //                                 scoringSubsystem.setOverrideKickerVoltsDirectly(
                //                                         0.0)));
                break;
        }
    }

    private void setUpDriveWithJoysticks() {
        if (FeatureFlags.runDrive) {
            drive.setDefaultCommand(
                    new DriveWithJoysticks(
                            drive,
                            leftJoystick,
                            rightJoystick,
                            PhoenixDriveConstants.maxSpeedMetPerSec,
                            PhoenixDriveConstants.MaxAngularRateRadPerSec,
                            DriverConstants.leftJoystickDeadband));
        }
    }
}
