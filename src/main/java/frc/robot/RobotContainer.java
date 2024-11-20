package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.ShootWithGamepad;
import frc.robot.constants.FeatureFlags;
import frc.robot.constants.ModeConstants;
import frc.robot.constants.PhoenixDriveConstants;
import frc.robot.constants.PhoenixDriveConstants.AlignTarget;
import frc.robot.constants.ScoringConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.OrchestraSubsystem;
import frc.robot.subsystems.drive.PhoenixDrive;
import frc.robot.subsystems.drive.PhoenixDrive.SysIdRoutineType;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeNEOVortex;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeAction;
import frc.robot.subsystems.localization.CameraContainerReal;
import frc.robot.subsystems.localization.CameraContainerReplay;
import frc.robot.subsystems.localization.CameraContainerSim;
import frc.robot.subsystems.localization.VisionLocalizer;
import frc.robot.subsystems.scoring.AimerIO;
import frc.robot.subsystems.scoring.AimerIORoboRio;
import frc.robot.subsystems.scoring.AimerIOSim;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringAction;
import frc.robot.subsystems.scoring.ShooterIO;
import frc.robot.subsystems.scoring.ShooterIOSim;
import frc.robot.subsystems.scoring.ShooterIOTalonFX;
import frc.robot.utils.AllianceUtil;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.feedforward.TuneG;
import frc.robot.utils.feedforward.TuneS;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class RobotContainer {
    PhoenixDrive drive;

    Telemetry logger;

    ScoringSubsystem scoringSubsystem;
    IntakeSubsystem intakeSubsystem;
    LED leds;

    OrchestraSubsystem orchestraSubsystem;

    CommandJoystick leftJoystick = new CommandJoystick(0);
    CommandJoystick rightJoystick = new CommandJoystick(1);
    CommandXboxController masher = new CommandXboxController(2);

    VisionLocalizer tagVision;

    SendableChooser<String> testModeChooser = new SendableChooser<String>();
    private LoggedTunableNumber shooterkP;
    private LoggedTunableNumber shooterkI;
    private LoggedTunableNumber shooterkD;

    private LoggedTunableNumber shooterkS;
    private LoggedTunableNumber shooterkV;
    private LoggedTunableNumber shooterkA;

    private LoggedTunableNumber tunableShooterRPM;
    private LoggedTunableNumber tunableShooterVolts;

    private LoggedTunableNumber aimerkP;
    private LoggedTunableNumber aimerkI;
    private LoggedTunableNumber aimerkD;

    private LoggedTunableNumber aimerkS;
    private LoggedTunableNumber aimerkV;
    private LoggedTunableNumber aimerkA;
    private LoggedTunableNumber aimerkG;

    private LoggedTunableNumber aimerProfileMaxVelocity;
    private LoggedTunableNumber aimerProfileMaxAcceleration;

    private LoggedTunableNumber tunableAimerPosition;
    private LoggedTunableNumber tunableAimerVolts;

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

        NamedCommands.registerCommand(
                "intakeNote",
                // intakes to start, ends by setting action to NONE when intake subsystem has note
                new InstantCommand(
                        () -> {
                            intakeSubsystem.run(IntakeAction.INTAKE);
                            scoringSubsystem.setAction(ScoringAction.INTAKE);
                        },
                        intakeSubsystem,
                        scoringSubsystem));

        NamedCommands.registerCommand(
                "waitForAlignment",
                Commands.waitUntil(() -> drive.isDriveAligned() && !scoringSubsystem.hasNote()));

        NamedCommands.registerCommand(
                "shootNoteAtSpeaker",
                new InstantCommand(() -> scoringSubsystem.setAction(ScoringAction.SHOOT)));
    }

    private void configureSubsystems() {
        if (FeatureFlags.runDrive) {
            initDrive();
        }
        if (FeatureFlags.runVision) {
            initVision();
        }
        if (FeatureFlags.runScoring) {
            initScoring();
        }
        if (FeatureFlags.runIntake) {
            initIntake();
        }
        if (FeatureFlags.runScoring && FeatureFlags.runIntake && FeatureFlags.runLEDS) {
            initLEDs();
        }
        if (FeatureFlags.runOrchestra) {
            initOrchestra();
        }
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
                tagVision = new VisionLocalizer(new CameraContainerReal(VisionConstants.cameras));
                break;
            case SIM:
                if (FeatureFlags.runDrive) {
                    tagVision =
                            new VisionLocalizer(
                                    new CameraContainerSim(
                                            VisionConstants.cameras, logger::getModuleStates));
                } else {
                    /* TODO: Maybe try to spoof vision sim without drive telemetry by giving it
                    all zeros */
                    throw new NullPointerException("Vision simulation depends on drive!");
                }
                break;
            case REPLAY:
                tagVision = new VisionLocalizer(new CameraContainerReplay(VisionConstants.cameras));
                break;
        }
    }

    private void initIntake() {
        switch (ModeConstants.currentMode) {
            case REAL:
                intakeSubsystem = new IntakeSubsystem(new IntakeNEOVortex());
                break;
            case SIM:
                intakeSubsystem = new IntakeSubsystem(new IntakeIOSim());
                break;
            case REPLAY:
                intakeSubsystem = new IntakeSubsystem(new IntakeIO() {});
                break;
        }
    }

    private void initScoring() {
        switch (ModeConstants.currentMode) {
            case REAL:
                scoringSubsystem =
                        new ScoringSubsystem(new ShooterIOTalonFX(), new AimerIORoboRio());
                break;
            case SIM:
                scoringSubsystem = new ScoringSubsystem(new ShooterIOSim(), new AimerIOSim());
                break;
            case REPLAY:
                scoringSubsystem = new ScoringSubsystem(new ShooterIO() {}, new AimerIO() {});
                break;
        }
    }

    private void initLEDs() {
        leds = new LED(scoringSubsystem, intakeSubsystem);
    }

    private void initOrchestra() {
        switch (ModeConstants.currentMode) {
            case REAL:
                orchestraSubsystem =
                        new OrchestraSubsystem(
                                "music/mii_channel.chrp"); // TODO: Add music files to deploy!
                if (FeatureFlags.runScoring) {
                    orchestraSubsystem.addInstruments(
                            scoringSubsystem.getAimerIO().getOrchestraMotors());
                    orchestraSubsystem.addInstruments(
                            scoringSubsystem.getShooterIO().getOrchestraMotors());
                }

                if (FeatureFlags.runDrive) {
                    orchestraSubsystem.addInstrument(drive.getModule(0).getDriveMotor());
                    orchestraSubsystem.addInstrument(drive.getModule(0).getSteerMotor());
                    orchestraSubsystem.addInstrument(drive.getModule(1).getDriveMotor());
                    orchestraSubsystem.addInstrument(drive.getModule(1).getSteerMotor());
                    orchestraSubsystem.addInstrument(drive.getModule(2).getDriveMotor());
                    orchestraSubsystem.addInstrument(drive.getModule(2).getSteerMotor());
                    orchestraSubsystem.addInstrument(drive.getModule(3).getDriveMotor());
                    orchestraSubsystem.addInstrument(drive.getModule(3).getSteerMotor());
                }
            default:
                break;
        }
    }

    private void configureSuppliers() {
        if (FeatureFlags.runScoring) {
            Supplier<Pose2d> poseSupplier;
            if (drive != null) {
                poseSupplier = () -> drive.getState().Pose;
            } else {
                poseSupplier = () -> new Pose2d();
            }

            scoringSubsystem.setPoseSupplier(poseSupplier);
            scoringSubsystem.setDriveAlignedSupplier(() -> drive.isDriveAligned());
        }

        if (FeatureFlags.runIntake) {
            BooleanSupplier shooterHasNote;
            BooleanSupplier shooterInIntakePosition;
            if (scoringSubsystem != null) {
                shooterHasNote =
                        () -> {
                            return scoringSubsystem.hasNote();
                        };
                shooterInIntakePosition =
                        () -> {
                            return scoringSubsystem.aimerAtIntakePosition();
                        };
            } else {
                shooterHasNote = () -> false;
                shooterInIntakePosition = () -> false;
            }
            intakeSubsystem.setShooterHasNoteSupplier(shooterHasNote);
            intakeSubsystem.setShooterAtIntakePosition(shooterInIntakePosition);
        }

        if (FeatureFlags.runVision) {
            if (drive != null) {
                tagVision.setCameraConsumer(
                        (m) -> drive.addVisionMeasurement(m.pose(), m.timestamp(), m.variance()));
            } else {
                tagVision.setCameraConsumer((m) -> {});
            }
        }

        if (FeatureFlags.runLEDS) {
            if (FeatureFlags.runVision) {
                // leds.setVisionWorkingSupplier(() -> tagVision.coprocessorConnected());
            } else {
                leds.setVisionWorkingSupplier(() -> false);
            }
        }
    }

    private void configureBindings() {
        if (drive != null) {
            drive.registerTelemetry(logger::telemeterize);
            drive.setDefaultCommand(new DriveWithJoysticks(drive, leftJoystick, rightJoystick));
        }
        if (DriverStation.isTest()) {
            // SYS ID
            if (drive != null) {
                // NOTE: These if-statements are nested because other subsystems will go here too

                /* Bindings for switching routines */
                /* DPad up = Translation; DPad down = Rotation; DPad right = Steer */

                masher.povDown()
                        .onTrue(
                                Commands.runOnce(
                                        () -> drive.setSysIdRoutine(SysIdRoutineType.Rotation),
                                        drive));
                masher.povRight()
                        .onTrue(
                                Commands.runOnce(
                                        () -> drive.setSysIdRoutine(SysIdRoutineType.Steer),
                                        drive));
                masher.povUp()
                        .onTrue(
                                Commands.runOnce(
                                        () -> drive.setSysIdRoutine(SysIdRoutineType.Translation),
                                        drive));

                /* Bindings for drivetrain characterization */
                /*
                 * These bindings require multiple buttons pushed to swap between quastatic and
                 * dynamic
                 */
                /*
                 * Back/Start select dynamic/quasistatic, Y/X select forward/reverse direction
                 */

                masher.back().and(masher.y()).whileTrue(drive.sysIdDynamic(Direction.kForward));
                masher.back().and(masher.x()).whileTrue(drive.sysIdDynamic(Direction.kReverse));
                masher.start()
                        .and(masher.y())
                        .whileTrue(drive.sysIdQuasistatic(Direction.kForward));
                masher.start()
                        .and(masher.x())
                        .whileTrue(drive.sysIdQuasistatic(Direction.kReverse));
            }
        }

        if (FeatureFlags.runIntake) {
            masher.b()
                    .onTrue(new InstantCommand(() -> intakeSubsystem.run(IntakeAction.INTAKE)))
                    .onFalse(new InstantCommand(() -> intakeSubsystem.run(IntakeAction.NONE)));

            masher.a()
                    .onTrue(new InstantCommand(() -> intakeSubsystem.run(IntakeAction.REVERSE)))
                    .onFalse(new InstantCommand(() -> intakeSubsystem.run(IntakeAction.NONE)));

            // HACK: This button was added during DCMP to un-jam the intake. Ideally, this
            // functionality should be implemented through a state machine.
            masher.x()
                    .onTrue(
                            new SequentialCommandGroup(
                                    new InstantCommand(
                                            () -> intakeSubsystem.run(IntakeAction.REVERSE)),
                                    Commands.waitSeconds(0.1),
                                    new InstantCommand(
                                            () -> intakeSubsystem.run(IntakeAction.INTAKE)),
                                    Commands.waitSeconds(0.5),
                                    new InstantCommand(
                                            () -> intakeSubsystem.run(IntakeAction.NONE))))
                    .onFalse(new InstantCommand(() -> intakeSubsystem.run(IntakeAction.NONE)));
        }

        if (FeatureFlags.runScoring) {
            scoringSubsystem.setDefaultCommand(
                    new ShootWithGamepad(
                            () -> rightJoystick.getHID().getRawButton(4),
                            masher.getHID()::getRightBumper,
                            masher.getHID()::getYButton,
                            () -> masher.getRightTriggerAxis() > 0.5,
                            masher.getHID()::getAButton,
                            masher.getHID()::getBButton,
                            scoringSubsystem,
                            () -> drive.getAlignTarget()));
            // FeatureFlags.runDrive ? drivetrain::getAlignTarget : () -> AlignTarget.NONE));

            rightJoystick
                    .button(11)
                    .onTrue(new InstantCommand(() -> scoringSubsystem.setArmDisabled(true)));
            rightJoystick
                    .button(16)
                    .onTrue(new InstantCommand(() -> scoringSubsystem.setArmDisabled(false)));

            rightJoystick
                    .button(12)
                    .onTrue(
                            new InstantCommand(
                                    () -> {
                                        scoringSubsystem.setAction(ScoringAction.OVERRIDE);
                                        scoringSubsystem.setVolts(3, 0);
                                    },
                                    scoringSubsystem));

            rightJoystick
                    .button(15)
                    .onTrue(
                            new InstantCommand(
                                    () -> {
                                        scoringSubsystem.setAction(ScoringAction.OVERRIDE);
                                        scoringSubsystem.setVolts(-3, 0);
                                    },
                                    scoringSubsystem));

            masher.povUp();
        }
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

            leftJoystick
                    .top()
                    .onTrue(
                            new InstantCommand(
                                    () ->
                                            drive.seedFieldRelative(
                                                    AllianceUtil.getPoseAgainstSpeaker())));

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
        intakeSubsystem.run(IntakeAction.NONE);
        scoringSubsystem.setAction(ScoringAction.WAIT);
    }

    public void onDSConnect() {
        drive.configurePathPlanner();
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

        shooterkP =
                new LoggedTunableNumber(
                        "Shooter-Tunables/PID/shooterkP", ScoringConstants.shooterkP);
        shooterkI =
                new LoggedTunableNumber(
                        "Shooter-Tunables/PID/shooterkI", ScoringConstants.shooterkI);
        shooterkD =
                new LoggedTunableNumber(
                        "Shooter-Tunables/PID/shooterkD", ScoringConstants.shooterkD);

        shooterkS =
                new LoggedTunableNumber(
                        "Shooter-Tunables/FF/shooterkS", ScoringConstants.shooterkS);
        shooterkV =
                new LoggedTunableNumber(
                        "Shooter-Tunables/FF/shooterkV", ScoringConstants.shooterkV);
        shooterkA =
                new LoggedTunableNumber(
                        "Shooter-Tunables/FF/shooterkA", ScoringConstants.shooterkA);

        tunableShooterRPM = new LoggedTunableNumber("Shooter-Tunables/shooterTunableRPM", 0.0);
        tunableShooterVolts = new LoggedTunableNumber("Shooter-Tunables/shooterTunableVolts", 0.0);

        aimerkP = new LoggedTunableNumber("Aimer-Tunables/PID/aimerkP", ScoringConstants.aimerkP);
        aimerkI = new LoggedTunableNumber("Aimer-Tunables/PID/aimerkI", ScoringConstants.aimerkI);
        aimerkD = new LoggedTunableNumber("Aimer-Tunables/PID/aimerkD", ScoringConstants.aimerkD);

        aimerkS = new LoggedTunableNumber("Aimer-Tunables/FF/aimerkS", ScoringConstants.aimerkS);
        aimerkV = new LoggedTunableNumber("Aimer-Tunables/FF/aimerkV", ScoringConstants.aimerkV);
        aimerkA = new LoggedTunableNumber("Aimer-Tunables/FF/aimerkA", ScoringConstants.aimerkA);
        aimerkG = new LoggedTunableNumber("Aimer-Tunables/FF/aimerkG", ScoringConstants.aimerkG);

        aimerProfileMaxVelocity =
                new LoggedTunableNumber(
                        "Aimer-Tunables/ProfileTuning/MaxVelocity",
                        ScoringConstants.aimerCruiseVelocity);
        aimerProfileMaxAcceleration =
                new LoggedTunableNumber(
                        "Aimer-Tunables/ProfileTuning/MaxAcceleration",
                        ScoringConstants.aimerAcceleration);

        tunableAimerPosition = new LoggedTunableNumber("Aimer-Tunables/aimerTunablePosition", 0.0);
        tunableAimerVolts = new LoggedTunableNumber("Aimer-Tunables/aimerTunableVolts", 0.0);

        switch (testModeChooser.getSelected()) {
            case "tuning":
                break;
            case "tuning-shooter":
                scoringSubsystem.setAction(ScoringAction.OVERRIDE);
                masher.a().onTrue(new TuneS(scoringSubsystem, 1));
                masher.b().onTrue(new TuneG(scoringSubsystem, 1));
                masher.y()
                        .onTrue(
                                new InstantCommand(
                                        () ->
                                                scoringSubsystem.runToPosition(
                                                        tunableShooterRPM.getAsDouble(), 1)))
                        .onTrue(
                                new InstantCommand(
                                        () ->
                                                scoringSubsystem.setAction(
                                                        ScoringAction.TEMPORARY_SETPOINT)))
                        .onFalse(
                                new InstantCommand(
                                        () -> scoringSubsystem.setAction(ScoringAction.OVERRIDE)));
                break;
            case "tuning-shot":
                scoringSubsystem.setAction(ScoringAction.OVERRIDE);
                setUpDriveWithJoysticks();
                masher.y()
                        .onTrue(
                                new InstantCommand(
                                        () ->
                                                scoringSubsystem.runToPosition(
                                                        tunableAimerPosition.getAsDouble(), 0)))
                        .onTrue(
                                new InstantCommand(
                                        () ->
                                                scoringSubsystem.runToPosition(
                                                        tunableShooterRPM.getAsDouble(), 1)))
                        .onTrue(
                                new InstantCommand(
                                        () ->
                                                scoringSubsystem.setAction(
                                                        ScoringAction.TEMPORARY_SETPOINT)))
                        .onFalse(
                                new InstantCommand(
                                        () -> scoringSubsystem.setAction(ScoringAction.OVERRIDE)));

                masher.leftBumper()
                        .onTrue(new InstantCommand(() -> scoringSubsystem.setTuningKickerVolts(12)))
                        .onFalse(
                                new InstantCommand(() -> scoringSubsystem.setTuningKickerVolts(0)));
                break;
            case "tuning-aimer":
                scoringSubsystem.setAction(ScoringAction.OVERRIDE);

                masher.a().onTrue(new TuneS(scoringSubsystem, 0));

                masher.b().onTrue(new TuneG(scoringSubsystem, 0));

                masher.y()
                        .onTrue(
                                new InstantCommand(
                                        () ->
                                                scoringSubsystem.runToPosition(
                                                        tunableAimerPosition.getAsDouble(), 0)))
                        .onTrue(
                                new InstantCommand(
                                        () ->
                                                scoringSubsystem.setAction(
                                                        ScoringAction.TEMPORARY_SETPOINT)))
                        .onFalse(
                                new InstantCommand(
                                        () -> scoringSubsystem.setAction(ScoringAction.OVERRIDE)));

                masher.leftBumper()
                        .onTrue(
                                new InstantCommand(
                                        () ->
                                                scoringSubsystem.setVolts(
                                                        tunableAimerVolts.getAsDouble(), 0)))
                        .onFalse(new InstantCommand(() -> scoringSubsystem.setVolts(0, 0)));

                break;
            case "tuning-amp":
                // Let us drive
                CommandScheduler.getInstance().cancelAll();
                setUpDriveWithJoysticks();

                // Let us intake
                masher.b()
                        .onTrue(new InstantCommand(() -> intakeSubsystem.run(IntakeAction.INTAKE)))
                        .onFalse(new InstantCommand(() -> intakeSubsystem.run(IntakeAction.NONE)));

                // Let us reverse intake (in case a note gets jammed during amp tuning)
                masher.a()
                        .onTrue(new InstantCommand(() -> intakeSubsystem.run(IntakeAction.REVERSE)))
                        .onFalse(new InstantCommand(() -> intakeSubsystem.run(IntakeAction.NONE)));

                masher.rightTrigger()
                        .onTrue(
                                new InstantCommand(
                                        () ->
                                                scoringSubsystem.runToPosition(
                                                        tunableAimerPosition.getAsDouble(), 0)))
                        .onTrue(
                                new InstantCommand(
                                        () ->
                                                scoringSubsystem.setAction(
                                                        ScoringAction.TEMPORARY_SETPOINT)))
                        .onFalse(
                                new InstantCommand(
                                        () -> scoringSubsystem.setAction(ScoringAction.OVERRIDE)));

                masher.rightBumper()
                        .onTrue(
                                new InstantCommand(
                                        () ->
                                                scoringSubsystem.setOverrideKickerVoltsDirectly(
                                                        12.0)))
                        .onFalse(
                                new InstantCommand(
                                        () ->
                                                scoringSubsystem.setOverrideKickerVoltsDirectly(
                                                        0.0)));
                break;
        }
    }

    public void testPeriodic() {
        LoggedTunableNumber.ifChanged(
                hashCode(),
                (pid) -> scoringSubsystem.setPID(pid[0], pid[1], pid[2], 1),
                shooterkP,
                shooterkI,
                shooterkD);
        LoggedTunableNumber.ifChanged(
                hashCode(),
                (FF) -> scoringSubsystem.setFF(FF[0], FF[1], FF[2], 0, 1),
                shooterkS,
                shooterkV,
                shooterkA);

        LoggedTunableNumber.ifChanged(
                hashCode(),
                (pid) -> scoringSubsystem.setPID(pid[0], pid[1], pid[2], 0),
                aimerkP,
                aimerkI,
                aimerkD);
        LoggedTunableNumber.ifChanged(
                hashCode(),
                (FF) -> scoringSubsystem.setFF(FF[0], FF[1], FF[2], FF[3], 0),
                aimerkS,
                aimerkV,
                aimerkA,
                aimerkG);
        LoggedTunableNumber.ifChanged(
                hashCode(),
                (MaxProfile) ->
                        scoringSubsystem.setMaxProfileProperties(MaxProfile[0], MaxProfile[1], 0),
                aimerProfileMaxVelocity,
                aimerProfileMaxAcceleration);
    }

    private void setUpDriveWithJoysticks() {
        if (FeatureFlags.runDrive) {
            drive.setDefaultCommand(new DriveWithJoysticks(drive, leftJoystick, rightJoystick));
        }
    }
}
