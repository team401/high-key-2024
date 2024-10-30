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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
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
    private SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    private Rotation2d goalRotation = new Rotation2d();
    private PIDController thetaController =
            new PIDController(
                    PhoenixDriveConstants.alignmentkP,
                    PhoenixDriveConstants.alignmentkI,
                    PhoenixDriveConstants.alignmentkD);

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

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.setTolerance(PhoenixDriveConstants.alignToleranceRadians);

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    public PhoenixDrive(
            SwerveDrivetrainConstants driveConstants, SwerveModuleConstants... modules) {
        super(driveConstants, modules);

        if (Utils.isSimulation()) {
            startSimThread();
        }

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.setTolerance(PhoenixDriveConstants.alignToleranceRadians);

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    public void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }
        Pathfinding.setPathfinder(new LocalADStar());

        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose,
                this::seedFieldRelative,
                this::getCurrentSpeeds,
                (speeds) -> {
                    this.setGoalSpeeds(speeds, false);
                },
                new HolonomicPathFollowerConfig(
                        new PIDConstants(0.1),
                        new PIDConstants(0.1, 0, 0),
                        PhoenixDriveConstants.kSpeedAt12VoltsMps,
                        driveBaseRadius,
                        new ReplanningConfig(false, false)),
                () -> DriverStation.getAlliance().get() == Alliance.Red,
                this);

        autoChooser.addOption("Nothing", Commands.none());
        autoChooser.setDefaultOption("Center Preload", new PathPlannerAuto("Center Preload"));
        autoChooser.addOption("Amp Side Preload", new PathPlannerAuto("Amp Side Preload"));
        autoChooser.addOption("Source Side Preload", new PathPlannerAuto("Source Side Preload"));
        autoChooser.addOption("Amp Side - 2 Note", new PathPlannerAuto("Amp Side - 2 Note"));
        autoChooser.addOption("Center - 4 Note", new PathPlannerAuto("Center - 4 Note"));
        autoChooser.addOption("Source Side - 3 Note", new PathPlannerAuto("Source Side - 3 Note"));
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAutoPath() {
        return autoChooser.getSelected();
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

        double omega = goalSpeeds.omegaRadiansPerSecond;
        if (aligning) {
            Rotation2d goalRotation = this.getAlignment().get();
            Logger.recordOutput("Drive/goalRotation", goalRotation);
            omega =
                    thetaController.calculate(
                            this.getState().Pose.getRotation().getRadians(),
                            goalRotation.getRadians());
            Logger.recordOutput("Drive/rotationError", thetaController.getPositionError());
        }

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
            request =
                    new SwerveRequest.FieldCentric()
                            .withVelocityX(goalSpeeds.vxMetersPerSecond)
                            .withVelocityY(goalSpeeds.vyMetersPerSecond)
                            .withRotationalRate(omega)
                            .withDeadband(0.0)
                            .withRotationalDeadband(0.0)
                            .withDriveRequestType(DriveRequestType.Velocity);

        } else {
            request =
                    new SwerveRequest.RobotCentric()
                            .withVelocityX(goalSpeeds.vxMetersPerSecond)
                            .withVelocityY(goalSpeeds.vyMetersPerSecond)
                            .withRotationalRate(omega)
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
        if (alignTarget == AlignTarget.NONE) {
            this.aligning = false;
        } else {
            this.aligning = aligning;
        }
    }

    public boolean isAligning() {
        return aligning;
    }

    // for scoring subsystem in auto
    public boolean isDriveAligned() {
        return thetaController.atSetpoint();
    }

    private Rotation2d getTargetHeading(Pose2d desiredTargetPose, boolean inverted) {
        Pose2d currentPose = this.getState().Pose;

        double targetVectorX = desiredTargetPose.getX() - currentPose.getX();
        double targetVectorY = desiredTargetPose.getY() - currentPose.getY();

        Rotation2d desiredRotation = new Rotation2d(targetVectorX, targetVectorY);

        if (inverted) {
            desiredRotation.minus(new Rotation2d(Math.PI));
        }
        return desiredRotation;
    }

    public Optional<Rotation2d> getAlignment() {
        switch (alignTarget) {
            case SPEAKER:
                if (!DriverStation.getAlliance().isEmpty()
                        && DriverStation.getAlliance().get() == Alliance.Blue) {
                    return Optional.of(
                            getTargetHeading(
                                    new Pose2d(FieldConstants.fieldToBlueSpeaker, new Rotation2d()),
                                    false));
                } else {
                    return Optional.of(
                            getTargetHeading(
                                    new Pose2d(FieldConstants.fieldToRedSpeaker, new Rotation2d()),
                                    true));
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
            case PASS:
                if (!DriverStation.getAlliance().isEmpty()
                        && DriverStation.getAlliance().get() == Alliance.Blue) {
                    return Optional.of(
                            getTargetHeading(
                                    new Pose2d(FieldConstants.fieldToBluePass, new Rotation2d()),
                                    false));
                } else {
                    return Optional.of(
                            getTargetHeading(
                                    new Pose2d(FieldConstants.fieldToRedPass, new Rotation2d()),
                                    true));
                }
            default:
                // no pose to align to so set target to none
                this.setAlignTarget(AlignTarget.NONE);
                this.setAligning(false);
                return Optional.empty();
        }
    }

    public void logDrivetrainData() {
        Logger.recordOutput("drive/pose", getState().Pose);

        SwerveDriveState state = getState();
        if (state.ModuleStates != null && state.ModuleTargets != null) {
            for (int i = 0; i < 4; i++) {
                Logger.recordOutput("Drive/module" + i + "/state", state.ModuleStates[i]);
                Logger.recordOutput("Drive/module" + i + "/target", state.ModuleTargets[i]);
            }
        }

        Logger.recordOutput("drive/alignment/alignTarget", alignTarget.toString());
        Logger.recordOutput("drive/alignment/isAligning", aligning);

        Logger.recordOutput(
                "drive/alignment/goalAlignment", goalRotation.plus(new Rotation2d(Math.PI)));
        Logger.recordOutput("drive/alignment/currentAlignemnt", getState().Pose.getRotation());
        Logger.recordOutput("drive/alignment/isDriveAligned", isDriveAligned());
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
        // sets request with velocity and rotational rate (alignment or right joystick)
        applyGoalSpeeds();
    }
}
