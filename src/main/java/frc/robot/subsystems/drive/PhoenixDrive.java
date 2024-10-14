package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Volts;

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
import frc.robot.constants.FieldConstants;
import frc.robot.constants.PhoenixDriveConstants;
import frc.robot.constants.PhoenixDriveConstants.AlignTarget;
import java.util.Optional;
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

    private AlignTarget alignTarget = AlignTarget.NONE;
    private boolean aligning = false;

    private ChassisSpeeds goalSpeeds = new ChassisSpeeds();
    private boolean fieldCentric = true;

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
                () -> DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue,
                this);

        PPHolonomicDriveController.setRotationTargetOverride(this::getAlignment);
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
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
        this.goalSpeeds = goalSpeeds;
        this.fieldCentric = fieldCentric;
    }

    public void applyGoalSpeeds() {
        SwerveRequest request;

        Logger.recordOutput("Drive/goalSpeeds", goalSpeeds);

        boolean idling =
                Math.sqrt(
                                        goalSpeeds.vxMetersPerSecond * goalSpeeds.vxMetersPerSecond
                                                + goalSpeeds.vyMetersPerSecond
                                                        * goalSpeeds.vyMetersPerSecond)
                                < 1e-10
                        && Math.abs(goalSpeeds.omegaRadiansPerSecond) < 1e-10;
        Logger.recordOutput("Drive/idling", idling);

        // sticks zeroed out will cause false idle when wanting alignment
        if (idling && !aligning) {
            request = new SwerveRequest.Idle();
        } else if (fieldCentric) {
            if (aligning) {
                SwerveRequest.FieldCentricFacingAngle alignRequest =
                        new SwerveRequest.FieldCentricFacingAngle()
                                .withVelocityX(goalSpeeds.vxMetersPerSecond)
                                .withVelocityY(goalSpeeds.vyMetersPerSecond)
                                .withTargetDirection(this.getAlignment().get())
                                .withDeadband(0.0)
                                .withRotationalDeadband(0.0)
                                .withDriveRequestType(DriveRequestType.Velocity);

                alignRequest.HeadingController.setPID(
                        PhoenixDriveConstants.alignmentkP,
                        PhoenixDriveConstants.alignmentkI,
                        PhoenixDriveConstants.alignmentkD);

                request = alignRequest;
            } else {
                request =
                        new SwerveRequest.FieldCentric()
                                .withVelocityX(goalSpeeds.vxMetersPerSecond)
                                .withVelocityY(goalSpeeds.vyMetersPerSecond)
                                .withRotationalRate(goalSpeeds.omegaRadiansPerSecond)
                                .withDeadband(0.0)
                                .withRotationalDeadband(0.0)
                                .withDriveRequestType(DriveRequestType.Velocity);
            }
        } else {
            request =
                    new SwerveRequest.RobotCentric()
                            .withVelocityX(goalSpeeds.vxMetersPerSecond)
                            .withVelocityY(goalSpeeds.vyMetersPerSecond)
                            .withRotationalRate(goalSpeeds.omegaRadiansPerSecond)
                            .withDeadband(0.0)
                            .withRotationalDeadband(0.0)
                            .withDriveRequestType(DriveRequestType.Velocity);
        }
        this.setControl(request);
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

    public void setAlignTarget(AlignTarget alignTarget) {
        this.alignTarget = alignTarget;
    }

    public AlignTarget getAlignTarget() {
        return alignTarget;
    }

    public void setAligning(boolean aligning) {
        this.aligning = aligning;
    }

    public boolean isAligning() {
        return aligning;
    }

    // for scoring subsystem in auto
    public boolean isDriveAligned() {
        if (alignTarget != null && aligning) {
            double desiredHeading = this.getAlignment().get().getRadians();
            double currentHeading = this.getState().Pose.getRotation().getRadians();

            if (Math.abs(desiredHeading - currentHeading)
                    < PhoenixDriveConstants.alignToleranceRadians) {
                return true;
            } else {
                return false;
            }
        } else {
            return false;
        }
    }

    private Rotation2d getTargetHeading(Pose2d desiredTargetPose) {
        Pose2d currentPose = this.getState().Pose;

        double targetVectorX = desiredTargetPose.getX() - currentPose.getX();
        double targetVectorY = desiredTargetPose.getY() - currentPose.getY();

        Rotation2d desiredRotation =
                new Rotation2d(targetVectorX, targetVectorY).minus(new Rotation2d(Math.PI));
        return desiredRotation;
    }

    public Optional<Rotation2d> getAlignment() {
        switch (alignTarget) {
            case SPEAKER:
                if (!DriverStation.getAlliance().isEmpty()
                        && DriverStation.getAlliance().get() == Alliance.Blue) {
                    return Optional.of(
                            getTargetHeading(
                                    new Pose2d(
                                            FieldConstants.fieldToBlueSpeaker, new Rotation2d())));
                } else {
                    return Optional.of(
                            getTargetHeading(
                                    new Pose2d(
                                            FieldConstants.fieldToRedSpeaker, new Rotation2d())));
                }
            case AMP:
                return Optional.of(FieldConstants.ampHeading);
            case SOURCE:
                if (DriverStation.getAlliance().get() == Alliance.Blue) {
                    return Optional.of(FieldConstants.blueSourceHeading);
                } else {
                    return Optional.of(FieldConstants.redSourceHeading);
                }
            case UP:
                if (DriverStation.getAlliance().get() == Alliance.Blue) {
                    return Optional.of(FieldConstants.blueUpHeading);
                } else {
                    return Optional.of(FieldConstants.redUpHeading);
                }
            case DOWN:
                if (DriverStation.getAlliance().get() == Alliance.Blue) {
                    return Optional.of(FieldConstants.blueDownHeading);
                } else {
                    return Optional.of(FieldConstants.redDownHeading);
                }
            case LEFT:
                if (DriverStation.getAlliance().get() == Alliance.Blue) {
                    return Optional.of(FieldConstants.blueLeftHeading);
                } else {
                    return Optional.of(FieldConstants.redLeftHeading);
                }
            case RIGHT:
                if (DriverStation.getAlliance().get() == Alliance.Blue) {
                    return Optional.of(FieldConstants.blueRightHeading);
                } else {
                    return Optional.of(FieldConstants.redRightHeading);
                }
            default:
                // no pose to align to so set target to none
                this.setAlignTarget(AlignTarget.NONE);
                return Optional.empty();
        }
    }

    public void logDrivetrainData() {
        SwerveDriveState state = getState();
        if (state.ModuleStates != null && state.ModuleTargets != null) {
            for (int i = 0; i < 4; i++) {
                Logger.recordOutput("Drive/module" + i + "/state", state.ModuleStates[i]);
                Logger.recordOutput("Drive/module" + i + "/target", state.ModuleTargets[i]);
            }
        }

        Logger.recordOutput("Align Target", alignTarget.toString());
    }

    @Override
    public void periodic() {
        logDrivetrainData();
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
        if (DriverStation.isTeleop()) {
            // sets request with velocity and rotational rate (alignment or right joystick)
            applyGoalSpeeds();
        }
    }
}
