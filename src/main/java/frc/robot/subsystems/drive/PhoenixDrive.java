package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.PhoenixDriveConstants;
import org.littletonrobotics.junction.Logger;

public class PhoenixDrive extends SwerveDrivetrain implements Subsystem {
    public enum SysIdRoutineType {
        Translation,
        Rotation,
        Steer
    };

    private Notifier simNotifier = null;
    private double lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;

    private final SwerveRequest.ApplyChassisSpeeds AutoRequest =
            new SwerveRequest.ApplyChassisSpeeds();

    private final SwerveRequest.SysIdSwerveTranslation TranslationCharacterization =
            new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveRotation RotationCharacterization =
            new SwerveRequest.SysIdSwerveRotation();
    private final SwerveRequest.SysIdSwerveSteerGains SteerCharacterization =
            new SwerveRequest.SysIdSwerveSteerGains();

    private SysIdRoutine SysIdRoutineTranslation =
            new SysIdRoutine(
                    new SysIdRoutine.Config(
                            null,
                            Volts.of(4),
                            null,
                            (state) -> SignalLogger.writeString("state", state.toString())),
                    new SysIdRoutine.Mechanism(
                            (volts) -> setControl(TranslationCharacterization.withVolts(volts)),
                            null,
                            this));

    private final SysIdRoutine SysIdRoutineRotation =
            new SysIdRoutine(
                    new SysIdRoutine.Config(
                            null,
                            Volts.of(4),
                            null,
                            (state) -> SignalLogger.writeString("state", state.toString())),
                    new SysIdRoutine.Mechanism(
                            (volts) -> setControl(RotationCharacterization.withVolts(volts)),
                            null,
                            this));

    private final SysIdRoutine SysIdRoutineSteer =
            new SysIdRoutine(
                    new SysIdRoutine.Config(
                            null,
                            Volts.of(7),
                            null,
                            (state) -> SignalLogger.writeString("state", state.toString())),
                    new SysIdRoutine.Mechanism(
                            (volts) -> setControl(SteerCharacterization.withVolts(volts)),
                            null,
                            this));

    /* Change this to the sysid routine you want to test */
    private SysIdRoutine routineToApply = SysIdRoutineTranslation;

    // auto
    private Pose2d desiredTargetPose;

    public PhoenixDrive(
            SwerveDrivetrainConstants driveConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveConstants, odometryUpdateFrequency, modules);

        configurePathPlanner();

        if (Utils.isSimulation()) {
            startSimThread();
        }

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    public PhoenixDrive(
            SwerveDrivetrainConstants driveConstants, SwerveModuleConstants... modules) {
        super(driveConstants, modules);

        configurePathPlanner();

        if (Utils.isSimulation()) {
            startSimThread();
        }

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }
        Pathfinding.setPathfinder(new LocalADStar());

        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose,
                this::seedFieldRelative,
                this::getCurrentSpeeds,
                (speeds) -> this.setControl(AutoRequest.withSpeeds(speeds)),
                new HolonomicPathFollowerConfig(
                        new PIDConstants(2),
                        new PIDConstants(
                                PhoenixDriveConstants.autoAlignmentkP,
                                PhoenixDriveConstants.autoAlignmentkI,
                                PhoenixDriveConstants.autoAlignmentkD),
                        PhoenixDriveConstants.kSpeedAt12VoltsMps,
                        driveBaseRadius,
                        new ReplanningConfig(false, false)),
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this);
        
        // override rotation for aiming at target
        PPHolonomicDriveController.setRotationTargetOverride(this::getAutoOverrideRotation);
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public void setAutoAlignTarget(Pose2d target) {
        this.desiredTargetPose = target;
    }

    // for use in auto pathplanner override
    private Optional<Rotation2d> getAutoOverrideRotation() {
        // TODO: add condition for if shooter has acquired note
        if (desiredTargetPose != null) {
            return Optional.of(desiredTargetPose.getRotation());
        } else {
            return Optional.empty();
        }
    }

    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();
        simNotifier =
                new Notifier(
                        () -> {
                            final double currentTime = Utils.getCurrentTimeSeconds();
                            double deltaTime = currentTime - lastSimTime;
                            lastSimTime = currentTime;

                            updateSimState(deltaTime, RobotController.getBatteryVoltage());
                        });

        simNotifier.startPeriodic(PhoenixDriveConstants.kSimLoopPeriod);
    }

    public ChassisSpeeds getCurrentSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public void setGoalSpeeds(ChassisSpeeds goalSpeeds, boolean fieldCentric) {
        SwerveRequest request;
        if (fieldCentric) {
            request =
                    new SwerveRequest.FieldCentric()
                            .withVelocityX(goalSpeeds.vxMetersPerSecond)
                            .withVelocityY(goalSpeeds.vyMetersPerSecond)
                            .withRotationalRate(goalSpeeds.omegaRadiansPerSecond)
                            .withDeadband(0.0)
                            .withRotationalDeadband(0.0)
                            .withDriveRequestType(DriveRequestType.Velocity);
        } else {
            request =
                    new SwerveRequest.RobotCentric()
                            .withVelocityX(goalSpeeds.vxMetersPerSecond)
                            .withVelocityY(goalSpeeds.vyMetersPerSecond)
                            .withRotationalRate(goalSpeeds.omegaRadiansPerSecond)
                            .withDeadband(0.0)
                            .withRotationalDeadband(0.0)
                            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        }
        this.setControl(request);
    }

    public void setTargetHeading (double targetHeading) {

    }

    // SYS ID

    public void setSysIdRoutine(SysIdRoutineType routineType) {
        switch (routineType) {
            case Translation:
                routineToApply = SysIdRoutineTranslation;
                Logger.recordOutput("Sys Id Routine Type", "Translation");
                break;
            case Steer:
                routineToApply = SysIdRoutineSteer;
                Logger.recordOutput("Sys Id Routine Type", "Steer");
                break;
            case Rotation:
                routineToApply = SysIdRoutineRotation;
                Logger.recordOutput("Sys Id Routine Type", "Rotation");
                break;
            default:
                return;
        }
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance()
                    .ifPresent(
                            (color) -> {
                                this.setOperatorPerspectiveForward(
                                        color == Alliance.Red
                                                ? RedAlliancePerspectiveRotation
                                                : BlueAlliancePerspectiveRotation);
                            });
            hasAppliedOperatorPerspective = true;
        }
    }
}
