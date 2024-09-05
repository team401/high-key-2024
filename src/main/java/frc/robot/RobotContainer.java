// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.constants.PhoenixDriveConstants;
import frc.robot.subsystems.drive.PhoenixDrive;
import frc.robot.subsystems.drive.commands.DriveWithJoysticks;
import frc.robot.constants.FeatureFlags;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.localization.CameraContainerReal;
import frc.robot.subsystems.localization.CameraContainerSim;
import frc.robot.subsystems.localization.VisionLocalizer;

public class RobotContainer {
    PhoenixDrive drive = PhoenixDriveConstants.DriveTrain;
    Telemetry logger = new Telemetry(6);
    CommandJoystick leftJoystick = new CommandJoystick(0);
    CommandJoystick rightJoystick = new CommandJoystick(1);

      VisionLocalizer tagVision;

    public RobotContainer() {
            configureSubsystems();
        configureBindings();
      }

      private void configureBindings() {
        drive.registerTelemetry(logger::telemeterize);
        drive.setDefaultCommand(new DriveWithJoysticks(drive, leftJoystick, rightJoystick));
    }

    private void configureSubsystems() {
        // TODO: Potentially migrate to Constants.mode
        if (Robot.isReal()) {
            if (FeatureFlags.runVision) {
                // TODO: Real robot vision
                tagVision = new VisionLocalizer(new CameraContainerReal(VisionConstants.cameras));
            }
        } else {
            if (FeatureFlags.simulateVision) {
                // TODO: Simulate robot vision
                tagVision = new VisionLocalizer(new CameraContainerSim(VisionConstants.cameras));
            }
        }
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
