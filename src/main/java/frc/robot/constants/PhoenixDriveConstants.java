package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class PhoenixDriveConstants {
    public enum AlignTarget {
        NONE,
        AMP,
        SPEAKER,
        SOURCE,
        ENDGAME,
        UP,
        DOWN,
        LEFT,
        RIGHT
    }

    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    public static final Slot0Configs steerGains =
            new Slot0Configs()
                    .withKP(150)
                    .withKI(50)
                    .withKD(0.2)
                    .withKS(0.25)
                    .withKV(1.5)
                    .withKA(0);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    public static final Slot0Configs driveGains =
            new Slot0Configs()
                    .withKP(0)
                    .withKI(0.02)
                    .withKD(0)
                    .withKS(0.26)
                    .withKV(0.12)
                    .withKA(0.01);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    private static final double kSlipCurrentA = 80;

    public static final double kSpeedAt12VoltsMps = 5.02; // 5.21 OR 5.02
    public static final double maxSpeedMetPerSec = 6;
    public static final double MaxAngularRateRadPerSec = Math.PI * 2;

    public static final double autoAlignmentkP = 5.0;
    public static final double autoAlignmentkI = 5.5;
    public static final double autoAlignmentkD = 0.0;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 3.5714285714285716;

    private static final double kDriveGearRatio = 6.122448979591837;
    private static final double kSteerGearRatio = 21.428571428571427;
    private static final double kWheelRadiusInches = 1.965;

    private static final boolean kSteerMotorReversed = true;
    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final String kCANbusName = "Canivore";
    private static final int kPigeonId = 1;

    // These are only used for simulation
    private static final double kSteerInertia = 0.00001;
    private static final double kDriveInertia = 0.001;
    // Simulated voltage necessary to overcome friction
    private static final double kSteerFrictionVoltage = 0.25;
    private static final double kDriveFrictionVoltage = 0.25;

    public static final SwerveDrivetrainConstants DrivetrainConstants =
            new SwerveDrivetrainConstants().withPigeon2Id(kPigeonId).withCANbusName(kCANbusName);

    private static final SwerveModuleConstantsFactory ConstantCreator =
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

    // sim constants
    public static final double kSimLoopPeriod = 0.005;

    // Front Left
    private static final int kBackRightDriveMotorId = 2;
    private static final int kBackRightSteerMotorId = 1;
    private static final int kBackRightEncoderId = 1;
    private static final double kBackRightEncoderOffset = 0.3486328125;

    private static final double kBackRightXPosInches = 10.375;
    private static final double kBackRightYPosInches = 10.375;

    // Front Right
    private static final int kBackLeftDriveMotorId = 4;
    private static final int kBackLeftSteerMotorId = 3;
    private static final int kBackLeftEncoderId = 2;
    private static final double kBackLeftEncoderOffset = 0.096435546875;

    private static final double kBackLeftXPosInches = 10.375;
    private static final double kBackLeftYPosInches = -10.375;

    // Back Left
    private static final int kFrontRightDriveMotorId = 8;
    private static final int kFrontRightSteerMotorId = 7;
    private static final int kFrontRightEncoderId = 4;
    private static final double kFrontRightEncoderOffset = 0.130859375;

    private static final double kFrontRightXPosInches = -10.375;
    private static final double kFrontRightYPosInches = 10.375;

    // Back Right
    private static final int kFrontLeftDriveMotorId = 6;
    private static final int kFrontLeftSteerMotorId = 5;
    private static final int kFrontLeftEncoderId = 3;
    private static final double kFrontLeftEncoderOffset = -0.372802734375;

    private static final double kFrontLeftXPosInches = -10.375;
    private static final double kFrontLeftYPosInches = -10.375;

    public static final double kModuleRadiusMeters =
            Units.inchesToMeters(Math.hypot(kFrontLeftXPosInches, kFrontLeftYPosInches));

    public static final SwerveModuleConstants FrontLeft =
            ConstantCreator.createModuleConstants(
                    kFrontLeftSteerMotorId,
                    kFrontLeftDriveMotorId,
                    kFrontLeftEncoderId,
                    kFrontLeftEncoderOffset,
                    Units.inchesToMeters(kFrontLeftXPosInches),
                    Units.inchesToMeters(kFrontLeftYPosInches),
                    kInvertLeftSide);
    public static final SwerveModuleConstants FrontRight =
            ConstantCreator.createModuleConstants(
                    kFrontRightSteerMotorId,
                    kFrontRightDriveMotorId,
                    kFrontRightEncoderId,
                    kFrontRightEncoderOffset,
                    Units.inchesToMeters(kFrontRightXPosInches),
                    Units.inchesToMeters(kFrontRightYPosInches),
                    kInvertRightSide);
    public static final SwerveModuleConstants BackLeft =
            ConstantCreator.createModuleConstants(
                    kBackLeftSteerMotorId,
                    kBackLeftDriveMotorId,
                    kBackLeftEncoderId,
                    kBackLeftEncoderOffset,
                    Units.inchesToMeters(kBackLeftXPosInches),
                    Units.inchesToMeters(kBackLeftYPosInches),
                    kInvertLeftSide);
    public static final SwerveModuleConstants BackRight =
            ConstantCreator.createModuleConstants(
                    kBackRightSteerMotorId,
                    kBackRightDriveMotorId,
                    kBackRightEncoderId,
                    kBackRightEncoderOffset,
                    Units.inchesToMeters(kBackRightXPosInches),
                    Units.inchesToMeters(kBackRightYPosInches),
                    kInvertRightSide);

    public static final SwerveDriveKinematics kinematics =
            new SwerveDriveKinematics(
                    new Translation2d(FrontLeft.LocationX, FrontLeft.LocationY),
                    new Translation2d(FrontLeft.LocationX, FrontRight.LocationY),
                    new Translation2d(BackLeft.LocationX, BackLeft.LocationY),
                    new Translation2d(BackRight.LocationX, BackRight.LocationY));
}
