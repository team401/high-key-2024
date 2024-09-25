package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
import frc.robot.utils.feedforward.TuneG;
import frc.robot.utils.feedforward.TuneS;

public class RobotContainer {
    PhoenixDrive drive;

    Telemetry logger;

    ScoringSubsystem scoringSubsystem;
    IntakeSubsystem intakeSubsystem;

    CommandJoystick leftJoystick = new CommandJoystick(0);
    CommandJoystick rightJoystick = new CommandJoystick(1);
    CommandXboxController masher = new CommandXboxController(2);

    VisionLocalizer tagVision;

    SendableChooser<String> testModeChooser = new SendableChooser<String>();

    public RobotContainer() {
        configureSubsystems();
        configureModes();
        configureBindings();
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

    private void configureModes() {
        testModeChooser.setDefaultOption("Blank", "tuning");

        testModeChooser.setDefaultOption("Aimer Tunig", "tuning-aimer");

        SmartDashboard.putData("Test Mode Chooser", testModeChooser);
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

        if (FeatureFlags.runIntake && Robot.isReal()
                || FeatureFlags.simulateIntake && !Robot.isReal()) {
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

        if (FeatureFlags.runScoring && Robot.isReal()
                || FeatureFlags.simulateScoring && !Robot.isReal()) {
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
        if (FeatureFlags.runDrive && Robot.isReal()
                || FeatureFlags.simulateDrive && !Robot.isReal()) {
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
                    .onTrue(new InstantCommand(() -> drive.setAlignTarget(AlignTarget.UP)));

            rightJoystick
                    .povDown()
                    .onTrue(new InstantCommand(() -> drive.setAlignTarget(AlignTarget.DOWN)));

            rightJoystick
                    .povLeft()
                    .onTrue(new InstantCommand(() -> drive.setAlignTarget(AlignTarget.LEFT)));

            rightJoystick
                    .povRight()
                    .onTrue(new InstantCommand(() -> drive.setAlignTarget(AlignTarget.RIGHT)));
        }
    } // spotless:on

    public void enabledInit() {
        intakeSubsystem.run(IntakeAction.NONE);
        scoringSubsystem.setAction(ScoringAction.WAIT);
    }

    public void testInit() {
        // Reset bindings
        masher = new CommandXboxController(2);

        switch (testModeChooser.getSelected()) {
            case "tuning":
                break;
            case "tuning-aimer":
                SmartDashboard.putNumber("Test-Mode/aimer/kP", ScoringConstants.aimerkP);
                SmartDashboard.putNumber("Test-Mode/aimer/kI", ScoringConstants.aimerkI);
                SmartDashboard.putNumber("Test-Mode/aimer/kD", ScoringConstants.aimerkD);

                SmartDashboard.putNumber(
                        "Test-Mode/aimer/profileMaxVelocity", ScoringConstants.aimerCruiseVelocity);
                SmartDashboard.putNumber(
                        "Test-Mode/aimer/profileMaxAcceleration",
                        ScoringConstants.aimerAcceleration);

                SmartDashboard.putNumber("Test-Mode/aimer/setpointPosition", 0.0);
                SmartDashboard.putNumber("Test-Mode/aimer/volts", 2.0);

                scoringSubsystem.setAction(ScoringAction.OVERRIDE);

                // TODO: Add Tunables to coppercore!
                masher.a().onTrue(new TuneS(scoringSubsystem, 0));

                masher.b().onTrue(new TuneG(scoringSubsystem, 0));

                masher.y()
                        .onTrue(
                                new InstantCommand(
                                        () ->
                                                scoringSubsystem.setPID(
                                                        SmartDashboard.getNumber(
                                                                "Test-Mode/aimer/kP",
                                                                ScoringConstants.aimerkP),
                                                        SmartDashboard.getNumber(
                                                                "Test-Mode/aimer/kI",
                                                                ScoringConstants.aimerkI),
                                                        SmartDashboard.getNumber(
                                                                "Test-Mode/aimer/kD",
                                                                ScoringConstants.aimerkD),
                                                        0)))
                        .onTrue(
                                new InstantCommand(
                                        () ->
                                                scoringSubsystem.setMaxProfileProperties(
                                                        SmartDashboard.getNumber(
                                                                "Test-Mode/aimer/profileMaxVelocity",
                                                                ScoringConstants
                                                                        .aimerCruiseVelocity),
                                                        SmartDashboard.getNumber(
                                                                "Test-Mode/aimer/profileMaxAcceleration",
                                                                ScoringConstants.aimerAcceleration),
                                                        0)))
                        .onTrue(
                                new InstantCommand(
                                        () ->
                                                scoringSubsystem.runToPosition(
                                                        SmartDashboard.getNumber(
                                                                "Test-Mode/aimer/setpointPosition",
                                                                0.0),
                                                        0)))
                        .onTrue(
                                new InstantCommand(
                                        () ->
                                                scoringSubsystem.setAction(
                                                        ScoringAction.TEMPORARY_SETPOINT)))
                        .onFalse(
                                new InstantCommand(
                                        () -> scoringSubsystem.setAction(ScoringAction.OVERRIDE)));

                // TODO: Figure out which of these we need
                // masher.povUp()
                //         .onTrue(new InstantCommand(() -> scoringSubsystem.runToPosition(1.1, 0)))
                //         .onTrue(
                //                 new InstantCommand(
                //                         () ->
                //                                 scoringSubsystem.setAction(
                //                                         ScoringAction.TEMPORARY_SETPOINT)))
                //         .onFalse(
                //                 new InstantCommand(
                //                         () ->
                // scoringSubsystem.setAction(ScoringAction.OVERRIDE)));

                // masher.povDown()
                //         .onTrue(new InstantCommand(() -> scoringSubsystem.runToPosition(0.0, 0)))
                //         .onTrue(
                //                 new InstantCommand(
                //                         () ->
                //                                 scoringSubsystem.setAction(
                //                                         ScoringAction.TEMPORARY_SETPOINT)))
                //         .onFalse(
                //                 new InstantCommand(
                //                         () ->
                // scoringSubsystem.setAction(ScoringAction.OVERRIDE)));

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
        }
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
