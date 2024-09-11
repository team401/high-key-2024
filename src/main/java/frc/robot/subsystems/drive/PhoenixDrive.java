package frc.robot.subsystems.drive;

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
import frc.robot.constants.PhoenixDriveConstants;

public class PhoenixDrive extends SwerveDrivetrain implements Subsystem {
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

        private SendableChooser<Command> autoChooser = new SendableChooser<Command>();

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
        // PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);

        // autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.setDefaultOption("Default (nothing)", Commands.none()); // S1-W1-W2-W3
        autoChooser.addOption(
                "Amp Side - 4 note (2 from center)", new PathPlannerAuto("S1-W1-C1-C2"));
        autoChooser.addOption(
                "Amp Side - 5 note (3 from center)", new PathPlannerAuto("S1-W1-C1-C2-C3"));
        autoChooser.addOption("Amp Side - 3 note", new PathPlannerAuto("S1-W1-W2"));
        autoChooser.addOption("Amp Side - 4 note (wing)", new PathPlannerAuto("S1-W1-W2-W3"));
        autoChooser.addOption("Amp Side - 5 note", new PathPlannerAuto("S1-W1-W2-W3-C5"));
        autoChooser.addOption("Center - 3 note", new PathPlannerAuto("S2-W2-W3"));
        autoChooser.addOption(
                "Center - 3 note (2 from center - avoids wing notes)",
                new PathPlannerAuto("S2-C1-C2"));
        autoChooser.addOption(
                "Center - 4 note (source side to center)", new PathPlannerAuto("S2-W2-W3-C5"));
        autoChooser.addOption("Center - 3 note - special", new PathPlannerAuto("S2-C1-C2-Special"));
        autoChooser.addOption(
                "Center - 5 note - 3 from center", new PathPlannerAuto("S2-C1-C2-C3"));
        autoChooser.addOption("Source Side - 2 note", new PathPlannerAuto("S3-W3"));
        autoChooser.addOption(
                "Source Side - 3 note - 2 from center", new PathPlannerAuto("S3-C5-C4"));
        autoChooser.addOption(
                "Source Side - 4 note - 3 from center", new PathPlannerAuto("S3-C5-C4-C3"));
        autoChooser.addOption(
                "Source Side - 5 note (across)", new PathPlannerAuto("S3-W3-W2-W1-C1"));
        autoChooser.addOption(
                "Source Side - 6 note (across)", new PathPlannerAuto("S3-W3-W2-W1-C1-C2"));
        SmartDashboard.putData("Auto Chooser", autoChooser);
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
    
    public Command getAutoCommand() {
        return autoChooser.getSelected();
    }
}
