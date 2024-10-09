package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
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
import frc.robot.constants.VisionConstants;
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

public class RobotContainer {
    PhoenixDrive drive;

    Telemetry logger;

    ScoringSubsystem scoringSubsystem;
    IntakeSubsystem intakeSubsystem;

    CommandJoystick leftJoystick = new CommandJoystick(0);
    CommandJoystick rightJoystick = new CommandJoystick(1);
    CommandXboxController masher = new CommandXboxController(2);

    VisionLocalizer tagVision;

    public RobotContainer() {
        configureSubsystems();
        configureNamedCommands();
        configureBindings();
    }

    private void configureNamedCommands() {
        NamedCommands.registerCommand(
                "alignToSpeaker",
                new InstantCommand(() -> drive.setAlignTarget(AlignTarget.SPEAKER)));

        NamedCommands.registerCommand(
                "intakeNote",
                // intakes to start, ends by setting action to NONE when intake subsystem has note
                new FunctionalCommand(
                        () -> intakeSubsystem.run(IntakeAction.INTAKE),
                        () -> {},
                        interrupted -> intakeSubsystem.run(IntakeAction.NONE),
                        () -> intakeSubsystem.hasNote(),
                        intakeSubsystem));

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
        if (FeatureFlags.runIntake) {
            initIntake();
        }
        if (FeatureFlags.runScoring) {
            initScoring();
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

        if (FeatureFlags.runVision) {
            tagVision.setCameraConsumer(
                    (m) -> drive.addVisionMeasurement(m.pose(), m.timestamp(), m.variance()));
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

    private void configureBindings() {
        if (drive != null) {
            drive.registerTelemetry(logger::telemeterize);
            drive.setDefaultCommand(new DriveWithJoysticks(drive, leftJoystick, rightJoystick));
            if (scoringSubsystem != null) {
                scoringSubsystem.setDriveAllignedSupplier(() -> drive.isDriveAligned());
            }
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
                    .onTrue(new InstantCommand(() -> drive.setAlignTarget(AlignTarget.ENDGAME)));

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
                    .onTrue(new InstantCommand(() -> drive.setAlignTarget(AlignTarget.ENDGAME)));

            rightJoystick
                    .trigger()
                    .onTrue(new InstantCommand(() -> drive.setAligning(true)))
                    .onFalse(new InstantCommand(() -> drive.setAligning(false)));
        }
    } // spotless:on

    public void enabledInit() {
        intakeSubsystem.run(IntakeAction.NONE);
        scoringSubsystem.setAction(ScoringAction.WAIT);
    }

    public Command getAutonomousCommand() {
        return drive.getAutoPath("Example");
    }
}
