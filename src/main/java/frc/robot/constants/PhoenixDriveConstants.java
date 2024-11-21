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

public  class PhoenixDriveConstants {
    public enum AlignTarget {
        NONE,
        AMP,
        SPEAKER,
        SOURCE,
        ENDGAME,
        UP,
        DOWN,
        LEFT,
        RIGHT,
        PASS
    }

    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    public   Slot0Configs steerGains =
            new Slot0Configs()
                    .withKP(150)
                    .withKI(50)
                    .withKD(0.2)
                    .withKS(0.25)
                    .withKV(1.5)
                    .withKA(0);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    public   Slot0Configs driveGains =
            new Slot0Configs()
                    .withKP(0.1)
                    .withKI(12.0)
                    .withKD(0.0)
                    .withKS(0.1)
                    .withKV(0.12)
                    .withKA(0.01);

    public   double alignmentkP = 4.0;
    public   double alignmentkI = 0.0;
    public   double alignmentkD = 0.0;
    public   double alignToleranceRadians = 0.1;

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private   ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private   ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    private   double kSlipCurrentA = 80;

    public   double kSpeedAt12VoltsMps = 5.02; // 5.21 OR 5.02
    public   double maxSpeedMetPerSec = 6;
    public   double MaxAngularRateRadPerSec = Math.PI * 2;

    public   double autoAlignmentkP = 5.0;
    public   double autoAlignmentkI = 5.5;
    public   double autoAlignmentkD = 0.0;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private   double kCoupleRatio = 3.5714285714285716;

    private   double kDriveGearRatio = 5.142857;
    private   double kSteerGearRatio = 25;
    private   double kWheelRadiusInches = 2;

    private   boolean kInvertLeftSide = false;
    private   boolean kInvertRightSide = true;

    private   String kCANbusName = "Canivore";
    private   int kPigeonId = 20;

    // These are only used for simulation
    private   double kSteerInertia = 0.00001;
    private   double kDriveInertia = 0.001;
    // Simulated voltage necessary to overcome friction
    private   double kSteerFrictionVoltage = 0.25;
    private   double kDriveFrictionVoltage = 0.25;

    public   SwerveDrivetrainConstants DrivetrainConstants =
            new SwerveDrivetrainConstants().withPigeon2Id(kPigeonId).withCANbusName(kCANbusName);

    private   SwerveModuleConstantsFactory ConstantCreator =
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
                    .withCouplingGearRatio(kCoupleRatio);

    // sim constants
    public   double kSimLoopPeriod = 0.005;

    // Front Left
    private   int kFrontLeftDriveMotorId = 7;
    private   int kFrontLeftSteerMotorId = 8;
    private   int kFrontLeftEncoderId = 9;
    private   double kFrontLeftEncoderOffset = -0.267822265625;
    private   boolean kFrontLeftSteerInvert = false;

    private   double kFrontLeftXPosInches = 10.75;
    private   double kFrontLeftYPosInches = 11.5;

    // Front Right
    private   int kFrontRightDriveMotorId = 1;
    private   int kFrontRightSteerMotorId = 2;
    private   int kFrontRightEncoderId = 12;
    private   double kFrontRightEncoderOffset = -0.228271484375;
    private   boolean kFrontRightSteerInvert = false;

    private   double kFrontRightXPosInches = 10.75;
    private   double kFrontRightYPosInches = -11.5;

    // Back Left
    private   int kBackLeftDriveMotorId = 5;
    private   int kBackLeftSteerMotorId = 6;
    private   int kBackLeftEncoderId = 10;
    private   double kBackLeftEncoderOffset = .084716796875;
    private   boolean kBackLeftSteerInvert = false;
    private   boolean kBackLeftDriveInvert = true;

    private   double kBackLeftXPosInches = -10.75;
    private   double kBackLeftYPosInches = 11.5;

    // Back Right
    private   int kBackRightDriveMotorId = 3;
    private   int kBackRightSteerMotorId = 4;
    private   int kBackRightEncoderId = 11;
    private   double kBackRightEncoderOffset = 0.39990234375;
    private   boolean kBackRightSteerInvert = true;

    private   double kBackRightXPosInches = -10.75;
    private   double kBackRightYPosInches = -11.5;

    public   double kModuleRadiusMeters =
            Units.inchesToMeters(Math.hypot(kFrontLeftXPosInches, kFrontLeftYPosInches));

    public   SwerveModuleConstants FrontLeft =
            ConstantCreator.createModuleConstants(
                            kFrontLeftSteerMotorId,
                            kFrontLeftDriveMotorId,
                            kFrontLeftEncoderId,
                            kFrontLeftEncoderOffset,
                            Units.inchesToMeters(kFrontLeftXPosInches),
                            Units.inchesToMeters(kFrontLeftYPosInches),
                            kInvertLeftSide)
                    .withSteerMotorInverted(kFrontLeftSteerInvert);
    public   SwerveModuleConstants FrontRight =
            ConstantCreator.createModuleConstants(
                            kFrontRightSteerMotorId,
                            kFrontRightDriveMotorId,
                            kFrontRightEncoderId,
                            kFrontRightEncoderOffset,
                            Units.inchesToMeters(kFrontRightXPosInches),
                            Units.inchesToMeters(kFrontRightYPosInches),
                            kInvertRightSide)
                    .withSteerMotorInverted(kFrontRightSteerInvert);
    public   SwerveModuleConstants BackLeft =
            ConstantCreator.createModuleConstants(
                            kBackLeftSteerMotorId,
                            kBackLeftDriveMotorId,
                            kBackLeftEncoderId,
                            kBackLeftEncoderOffset,
                            Units.inchesToMeters(kBackLeftXPosInches),
                            Units.inchesToMeters(kBackLeftYPosInches),
                            kInvertLeftSide)
                    .withSteerMotorInverted(kBackLeftSteerInvert)
                    .withDriveMotorInverted(kBackLeftDriveInvert);
    public   SwerveModuleConstants BackRight =
            ConstantCreator.createModuleConstants(
                            kBackRightSteerMotorId,
                            kBackRightDriveMotorId,
                            kBackRightEncoderId,
                            kBackRightEncoderOffset,
                            Units.inchesToMeters(kBackRightXPosInches),
                            Units.inchesToMeters(kBackRightYPosInches),
                            kInvertRightSide)
                    .withSteerMotorInverted(kBackRightSteerInvert);

    public   SwerveDriveKinematics kinematics =
            new SwerveDriveKinematics(
                    new Translation2d(FrontLeft.LocationX, FrontLeft.LocationY),
                    new Translation2d(FrontLeft.LocationX, FrontRight.LocationY),
                    new Translation2d(BackLeft.LocationX, BackLeft.LocationY),
                    new Translation2d(BackRight.LocationX, BackRight.LocationY));
}
