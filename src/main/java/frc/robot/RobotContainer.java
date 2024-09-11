// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.constants.PhoenixDriveConstants;
import frc.robot.subsystems.drive.PhoenixDrive;
import frc.robot.subsystems.drive.commands.DriveWithJoysticks;

public class RobotContainer {
    PhoenixDrive drive = PhoenixDriveConstants.DriveTrain;
    Telemetry logger = new Telemetry(6);
    CommandJoystick leftJoystick = new CommandJoystick(0);
    CommandJoystick rightJoystick = new CommandJoystick(1);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        drive.registerTelemetry(logger::telemeterize);
        drive.setDefaultCommand(new DriveWithJoysticks(drive, leftJoystick, rightJoystick));
    }

    private void configureAutonomous() {
        /*if (true) {
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
        }*/
    }


    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
