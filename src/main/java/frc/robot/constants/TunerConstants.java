package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import coppercore.parameter_tools.JSONExclude;
import coppercore.parameter_tools.JSONSync;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

// import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

// Time is in seconds
// Distance is in meters
// Angle is in radians
// Speed is in meters per second
public final class TunerConstants {

    @JSONExclude
    public static final JSONSync<TunerConstants> synced =
            new JSONSync<TunerConstants>(
                    new TunerConstants(), "filePath", new JSONSync.JSONSyncConfigBuilder().build());

    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with
    // the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    @JSONExclude
    public final Slot0Configs steerGains =
            new Slot0Configs()
                    .withKP(150)
                    .withKI(50)
                    .withKD(0.2)
                    .withKS(0.25)
                    .withKV(1.5)
                    .withKA(0);

    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    @JSONExclude
    public final Slot0Configs driveGains =
            new Slot0Configs()
                    .withKP(0)
                    .withKI(0.02)
                    .withKD(0)
                    .withKS(0.26)
                    .withKV(0.12)
                    .withKA(0.01);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private final double kSlipCurrentA = 80;

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    public final double kSpeedAt12VoltsMps = 5.02; // 5.21 OR 5.02

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private final double kCoupleRatio = 3.5714285714285716;

    private final double kDriveGearRatio = 6.122448979591837;
    private final double kSteerGearRatio = 21.428571428571427;
    private final double kWheelRadiusInches = 1.965;

    private final boolean kSteerMotorReversed = true;
    private final boolean kInvertLeftSide = false;
    private final boolean kInvertRightSide = true;

    private final String kCANbusName = "Canivore";
    private final int kPigeonId = 1;

    // These are only used for simulation
    private final double kSteerInertia = 0.00001;
    private final double kDriveInertia = 0.001;
    // Simulated voltage necessary to overcome friction
    private final double kSteerFrictionVoltage = 0.25;
    private final double kDriveFrictionVoltage = 0.25;

    @JSONExclude
    private final SwerveDrivetrainConstants DrivetrainConstants =
            new SwerveDrivetrainConstants().withPigeon2Id(kPigeonId).withCANbusName(kCANbusName);

    @JSONExclude
    private final SwerveModuleConstantsFactory ConstantCreator =
            new SwerveModuleConstantsFactory()
                    .withDriveMotorGearRatio(kDriveGearRatio)
                    .withSteerMotorGearRatio(kSteerGearRatio)
                    .withWheelRadius(kWheelRadiusInches)
                    .withSlipCurrent(kSlipCurrentA)
                    .withSteerMotorGains(steerGains)
                    .withDriveMotorGains(driveGains)
                    .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
                    .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
                    .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
                    .withSteerInertia(kSteerInertia)
                    .withDriveInertia(kDriveInertia)
                    .withSteerFrictionVoltage(kSteerFrictionVoltage)
                    .withDriveFrictionVoltage(kDriveFrictionVoltage)
                    .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
                    .withCouplingGearRatio(kCoupleRatio)
                    .withSteerMotorInverted(kSteerMotorReversed);

    // Front Left
    private final int kBackRightDriveMotorId = 2;
    private final int kBackRightSteerMotorId = 1;
    private final int kBackRightEncoderId = 1;
    private final double kBackRightEncoderOffset = 0.3486328125;

    private final double kBackRightXPosInches = 10.375;
    private final double kBackRightYPosInches = 10.375;

    // Front Right
    private final int kBackLeftDriveMotorId = 4;
    private final int kBackLeftSteerMotorId = 3;
    private final int kBackLeftEncoderId = 2;
    private final double kBackLeftEncoderOffset = 0.096435546875;

    private final double kBackLeftXPosInches = 10.375;
    private final double kBackLeftYPosInches = -10.375;

    // Back Left
    private final int kFrontRightDriveMotorId = 8;
    private final int kFrontRightSteerMotorId = 7;
    private final int kFrontRightEncoderId = 4;
    private final double kFrontRightEncoderOffset = 0.130859375;

    private final double kFrontRightXPosInches = -10.375;
    private final double kFrontRightYPosInches = 10.375;

    // Back Right
    private final int kFrontLeftDriveMotorId = 6;
    private final int kFrontLeftSteerMotorId = 5;
    private final int kFrontLeftEncoderId = 3;
    private final double kFrontLeftEncoderOffset = -0.372802734375;

    private final double kFrontLeftXPosInches = -10.375;
    private final double kFrontLeftYPosInches = -10.375;

    public final double kModuleRadiusMeters =
            Units.inchesToMeters(Math.hypot(kFrontLeftXPosInches, kFrontLeftYPosInches));

    @JSONExclude
    private final SwerveModuleConstants FrontLeft =
            ConstantCreator.createModuleConstants(
                    kFrontLeftSteerMotorId,
                    kFrontLeftDriveMotorId,
                    kFrontLeftEncoderId,
                    kFrontLeftEncoderOffset,
                    Units.inchesToMeters(kFrontLeftXPosInches),
                    Units.inchesToMeters(kFrontLeftYPosInches),
                    kInvertLeftSide);

    @JSONExclude
    private final SwerveModuleConstants FrontRight =
            ConstantCreator.createModuleConstants(
                    kFrontRightSteerMotorId,
                    kFrontRightDriveMotorId,
                    kFrontRightEncoderId,
                    kFrontRightEncoderOffset,
                    Units.inchesToMeters(kFrontRightXPosInches),
                    Units.inchesToMeters(kFrontRightYPosInches),
                    kInvertRightSide);

    @JSONExclude
    private final SwerveModuleConstants BackLeft =
            ConstantCreator.createModuleConstants(
                    kBackLeftSteerMotorId,
                    kBackLeftDriveMotorId,
                    kBackLeftEncoderId,
                    kBackLeftEncoderOffset,
                    Units.inchesToMeters(kBackLeftXPosInches),
                    Units.inchesToMeters(kBackLeftYPosInches),
                    kInvertLeftSide);

    @JSONExclude
    private final SwerveModuleConstants BackRight =
            ConstantCreator.createModuleConstants(
                    kBackRightSteerMotorId,
                    kBackRightDriveMotorId,
                    kBackRightEncoderId,
                    kBackRightEncoderOffset,
                    Units.inchesToMeters(kBackRightXPosInches),
                    Units.inchesToMeters(kBackRightYPosInches),
                    kInvertRightSide);

    @JSONExclude
    public final SwerveDriveKinematics kinematics =
            new SwerveDriveKinematics(
                    new Translation2d(FrontLeft.LocationX, FrontLeft.LocationY),
                    new Translation2d(FrontLeft.LocationX, FrontRight.LocationY),
                    new Translation2d(BackLeft.LocationX, BackLeft.LocationY),
                    new Translation2d(BackRight.LocationX, BackRight.LocationY));

    // public static final CommandSwerveDrivetrain DriveTrain = new CommandSwerveDrivetrain(
    //                 DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight);
}
