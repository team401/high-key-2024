package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.ShootWithGamepad;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.AlignTarget;
import frc.robot.constants.Constants.Mode;
import frc.robot.constants.FeatureFlags;
import frc.robot.constants.PhoenixDriveConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drive.PhoenixDrive;
import frc.robot.subsystems.drive.PhoenixDrive.SysIdRoutineType;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeNEOVortex;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeAction;
import frc.robot.subsystems.localization.CameraContainerReal;
import frc.robot.subsystems.localization.CameraContainerSim;
import frc.robot.subsystems.localization.VisionLocalizer;
import frc.robot.subsystems.scoring.AimerIORoboRio;
import frc.robot.subsystems.scoring.AimerIOSim;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringAction;
import frc.robot.subsystems.scoring.ShooterIOSim;
import frc.robot.subsystems.scoring.ShooterIOTalonFX;

public class RobotContainer {
    PhoenixDrive drive = PhoenixDriveConstants.DriveTrain;
    Telemetry logger = new Telemetry(6);

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
        // example command to set align target for auto override rotation
        // if (!DriverStation.getAlliance().isEmpty()) {
        //     switch (DriverStation.getAlliance().get()) {
        //         case Blue:
        //             NamedCommands.registerCommand(
        //                     "alignToSpeaker",
        //                     new AlignToTarget(
        //                             drive,
        //                             new Pose2d(
        //                                     FieldConstants.fieldToBlueSpeaker, new
        // Rotation2d())));
        //             return;
        //         case Red:
        //             NamedCommands.registerCommand(
        //                     "alignToSpeaker",
        //                     new AlignToTarget(
        //                             drive,
        //                             new Pose2d(
        //                                     FieldConstants.fieldToRedSpeaker, new
        // Rotation2d())));
        //             return;
        //     }
        // } else {
        //     NamedCommands.registerCommand(
        //             "alignToSpeaker",
        //             new AlignToTarget(
        //                     drive, new Pose2d(FieldConstants.fieldToRedSpeaker, new
        // Rotation2d())));
        // }
        NamedCommands.registerCommand(
                "alignToSpeaker",
                new InstantCommand(() -> drive.setAlignTarget(AlignTarget.SPEAKER)));
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

        if (Constants.FeatureFlags.runIntake) {
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

        if (Constants.FeatureFlags.runScoring) {

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
        if (Constants.FeatureFlags.runDrive) {
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
        if (Constants.currentMode == Mode.REAL) {
            scoringSubsystem = new ScoringSubsystem(new ShooterIOTalonFX(), new AimerIORoboRio());
            intakeSubsystem = new IntakeSubsystem(new IntakeNEOVortex());
        } else if (Constants.currentMode == Mode.SIM) {
            scoringSubsystem = new ScoringSubsystem(new ShooterIOSim(), new AimerIOSim());
            intakeSubsystem = new IntakeSubsystem(new IntakeIOSim());
        }
    }

    public void enabledInit() {
        intakeSubsystem.run(IntakeAction.NONE);
        scoringSubsystem.setAction(ScoringAction.WAIT);
    }

    public Command getAutonomousCommand() {
        return drive.getAutoPath("Example");
    }
}
