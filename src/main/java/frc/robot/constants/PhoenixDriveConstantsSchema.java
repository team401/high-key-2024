package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import coppercore.parameter_tools.JSONExclude;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public  class PhoenixDriveConstantsSchema {
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

    public double steerKP = 150;
    public double steerKI = 50;
    public double steerKD = 0.2;
    public double steerKS = 0.25;
    public double steerKV = 1.5;
    public double steerKA = 0;

    public double driveKP = 0.1;
    public double driveKI = 12.0;
    public double driveKD = 0.0;
    public double driveKS = 0.1;
    public double driveKV = 0.12;
    public double driveKA = 0.01;


    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    @JSONExclude public   Slot0Configs steerGains =
            new Slot0Configs()
                    .withKP(150)
                    .withKI(50)
                    .withKD(0.2)
                    .withKS(0.25)
                    .withKV(1.5)
                    .withKA(0);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    @JSONExclude public   Slot0Configs driveGains =
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
    @JSONExclude public   ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    @JSONExclude public   ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    public   double kSlipCurrentA = 80;

    public   double kSpeedAt12VoltsMps = 5.02; // 5.21 OR 5.02
    public   double maxSpeedMetPerSec = 6;
    public   double MaxAngularRateRadPerSec = Math.PI * 2;

    public   double autoAlignmentkP = 5.0;
    public   double autoAlignmentkI = 5.5;
    public   double autoAlignmentkD = 0.0;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    public   double kCoupleRatio = 3.5714285714285716;

    public   double kDriveGearRatio = 5.142857;
    public   double kSteerGearRatio = 25;
    public   double kWheelRadiusInches = 2;

    public   boolean kInvertLeftSide = false;
    public   boolean kInvertRightSide = true;

    public   String kCANbusName = "Canivore";
    public   int kPigeonId = 20;

    // These are only used for simulation
    public   double kSteerInertia = 0.00001;
    public   double kDriveInertia = 0.001;
    // Simulated voltage necessary to overcome friction
    public   double kSteerFrictionVoltage = 0.25;
    public   double kDriveFrictionVoltage = 0.25;

    @JSONExclude public   SwerveDrivetrainConstants DrivetrainConstants =
            new SwerveDrivetrainConstants().withPigeon2Id(kPigeonId).withCANbusName(kCANbusName);

    @JSONExclude public   SwerveModuleConstantsFactory ConstantCreator =
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
    public   int kFrontLeftDriveMotorId = 7;
    public   int kFrontLeftSteerMotorId = 8;
    public   int kFrontLeftEncoderId = 9;
    public   double kFrontLeftEncoderOffset = -0.267822265625;
    public   boolean kFrontLeftSteerInvert = false;

    public   double kFrontLeftXPosInches = 10.75;
    public   double kFrontLeftYPosInches = 11.5;

    // Front Right
    public   int kFrontRightDriveMotorId = 1;
    public   int kFrontRightSteerMotorId = 2;
    public   int kFrontRightEncoderId = 12;
    public   double kFrontRightEncoderOffset = -0.228271484375;
    public   boolean kFrontRightSteerInvert = false;

    public   double kFrontRightXPosInches = 10.75;
    public   double kFrontRightYPosInches = -11.5;

    // Back Left
    public   int kBackLeftDriveMotorId = 5;
    public   int kBackLeftSteerMotorId = 6;
    public   int kBackLeftEncoderId = 10;
    public   double kBackLeftEncoderOffset = .084716796875;
    public   boolean kBackLeftSteerInvert = false;
    public   boolean kBackLeftDriveInvert = true;

    public   double kBackLeftXPosInches = -10.75;
    public   double kBackLeftYPosInches = 11.5;

    // Back Right
    public   int kBackRightDriveMotorId = 3;
    public   int kBackRightSteerMotorId = 4;
    public   int kBackRightEncoderId = 11;
    public   double kBackRightEncoderOffset = 0.39990234375;
    public   boolean kBackRightSteerInvert = true;

    public   double kBackRightXPosInches = -10.75;
    public   double kBackRightYPosInches = -11.5;

    public   double kModuleRadiusMeters =
            Units.inchesToMeters(Math.hypot(kFrontLeftXPosInches, kFrontLeftYPosInches));

    @JSONExclude public   SwerveModuleConstants FrontLeft =
            ConstantCreator.createModuleConstants(
                            kFrontLeftSteerMotorId,
                            kFrontLeftDriveMotorId,
                            kFrontLeftEncoderId,
                            kFrontLeftEncoderOffset,
                            Units.inchesToMeters(kFrontLeftXPosInches),
                            Units.inchesToMeters(kFrontLeftYPosInches),
                            kInvertLeftSide)
                    .withSteerMotorInverted(kFrontLeftSteerInvert);
    @JSONExclude public   SwerveModuleConstants FrontRight =
            ConstantCreator.createModuleConstants(
                            kFrontRightSteerMotorId,
                            kFrontRightDriveMotorId,
                            kFrontRightEncoderId,
                            kFrontRightEncoderOffset,
                            Units.inchesToMeters(kFrontRightXPosInches),
                            Units.inchesToMeters(kFrontRightYPosInches),
                            kInvertRightSide)
                    .withSteerMotorInverted(kFrontRightSteerInvert);
    @JSONExclude public   SwerveModuleConstants BackLeft =
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
    @JSONExclude public   SwerveModuleConstants BackRight =
            ConstantCreator.createModuleConstants(
                            kBackRightSteerMotorId,
                            kBackRightDriveMotorId,
                            kBackRightEncoderId,
                            kBackRightEncoderOffset,
                            Units.inchesToMeters(kBackRightXPosInches),
                            Units.inchesToMeters(kBackRightYPosInches),
                            kInvertRightSide)
                    .withSteerMotorInverted(kBackRightSteerInvert);

    @JSONExclude public   SwerveDriveKinematics kinematics =
            new SwerveDriveKinematics(
                    new Translation2d(FrontLeft.LocationX, FrontLeft.LocationY),
                    new Translation2d(FrontLeft.LocationX, FrontRight.LocationY),
                    new Translation2d(BackLeft.LocationX, BackLeft.LocationY),
                    new Translation2d(BackRight.LocationX, BackRight.LocationY));
}
