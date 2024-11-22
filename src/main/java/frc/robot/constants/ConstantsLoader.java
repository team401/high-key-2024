package frc.robot.constants;

import java.util.HashMap;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;

import coppercore.parameter_tools.JSONSync;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class ConstantsLoader {
    /*
     * NOTE: Each json constants file needs a schema matching its structure
     *
     * -> that schema is then put into a new JSON Sync class,
     *      class' load method is called,
     *      and then object is retrieved for holding in corresponding static variable
     *
     * IMPORTANT: file paths are from base folder (High-Key-2024)
     */
    public static DriverConstantsSchema DriverConstants;
    public static FeatureFlagsSchema FeatureFlags;
    public static ScoringConstantsSchema ScoringConstants;
    public static PhoenixDriveConstantsSchema PhoenixDriveConstants;

    public static void loadDriverConstants() {
        JSONSync<DriverConstantsSchema> synced =
                new JSONSync<DriverConstantsSchema>(
                        new DriverConstantsSchema(),
                        "./DriverConstants.json",
                        new JSONSync.JSONSyncConfigBuilder().build());
        synced.loadData();
        DriverConstants = synced.getObject();
    }

    public static void loadFeatureFlags() {
        JSONSync<FeatureFlagsSchema> synced =
                new JSONSync<FeatureFlagsSchema>(
                        new FeatureFlagsSchema(),
                        "./DriverConstants.json",
                        new JSONSync.JSONSyncConfigBuilder().build());
        synced.loadData();
        FeatureFlags = synced.getObject();
    }

    public static void loadScoringConstants() {
        JSONSync<ScoringConstantsSchema> synced =
                new JSONSync<ScoringConstantsSchema>(
                        new ScoringConstantsSchema(),
                        "./DriverConstants.json",
                        new JSONSync.JSONSyncConfigBuilder().build());
        synced.loadData();
        ScoringConstants = synced.getObject();
        ScoringConstants.aimerMap = new HashMap<Double, Double>();
        ScoringConstants.shooterMap = new HashMap<Double, Double>();
        ScoringConstants.timeToGoalMap = new HashMap<Double, Double>();
        ScoringConstants.aimerToleranceMap = new HashMap<Double, Double>();

        for (int i = 0; i < ScoringConstants.aimerDistance.length; i++) {
            ScoringConstants.aimerMap.put(ScoringConstants.aimerDistance[i], ScoringConstants.aimerPosition[i]);
        }

        for (int i = 0; i < ScoringConstants.shooterDistance.length; i++) {
            ScoringConstants.shooterMap.put(ScoringConstants.shooterDistance[i], ScoringConstants.shooterRPM[i]);
        }

        for (int i = 0; i < ScoringConstants.timeToGoalDistance.length; i++) {
            ScoringConstants.timeToGoalMap.put(ScoringConstants.timeToGoalDistance[i], ScoringConstants.timeToGoal[i]);
        }

        for (int i = 0; i < ScoringConstants.aimerToleranceDistance.length; i++) {
            ScoringConstants.aimerToleranceMap.put(ScoringConstants.aimerToleranceDistance[i], ScoringConstants.aimerTolerance[i]);
        }
    }

    public static void loadPhoenixDriveConstants() {
                JSONSync<PhoenixDriveConstantsSchema> synced =
                new JSONSync<PhoenixDriveConstantsSchema>(
                        new PhoenixDriveConstantsSchema(),
                        "./DriverConstants.json",
                        new JSONSync.JSONSyncConfigBuilder().build());
        synced.loadData();
        PhoenixDriveConstants = synced.getObject();

        PhoenixDriveConstants.steerGains =
            new Slot0Configs()
                    .withKP(PhoenixDriveConstants.steerKP)
                    .withKI(PhoenixDriveConstants.steerKI)
                    .withKD(PhoenixDriveConstants.steerKD)
                    .withKS(PhoenixDriveConstants.steerKS)
                    .withKV(PhoenixDriveConstants.steerKV)
                    .withKA(PhoenixDriveConstants.steerKA);

        PhoenixDriveConstants.driveGains =
        new Slot0Configs()
                .withKP(PhoenixDriveConstants.driveKP)
                .withKI(PhoenixDriveConstants.driveKI)
                .withKD(PhoenixDriveConstants.driveKD)
                .withKS(PhoenixDriveConstants.driveKS)
                .withKV(PhoenixDriveConstants.driveKV)
                .withKA(PhoenixDriveConstants.driveKA);

        PhoenixDriveConstants.DrivetrainConstants =
            new SwerveDrivetrainConstants().withPigeon2Id(PhoenixDriveConstants.kPigeonId).withCANbusName(PhoenixDriveConstants.kCANbusName);

        PhoenixDriveConstants.ConstantCreator =
            new SwerveModuleConstantsFactory()
                    .withDriveMotorGearRatio(PhoenixDriveConstants.kDriveGearRatio)
                    .withSteerMotorGearRatio(PhoenixDriveConstants.kSteerGearRatio)
                    .withWheelRadius(PhoenixDriveConstants.kWheelRadiusInches)
                    .withSlipCurrent(PhoenixDriveConstants.kSlipCurrentA)
                    .withSteerMotorGains(PhoenixDriveConstants.steerGains)
                    .withDriveMotorGains(PhoenixDriveConstants.driveGains)
                    .withSteerMotorClosedLoopOutput(PhoenixDriveConstants.steerClosedLoopOutput)
                    .withDriveMotorClosedLoopOutput(PhoenixDriveConstants.driveClosedLoopOutput)
                    .withSpeedAt12VoltsMps(PhoenixDriveConstants.kSpeedAt12VoltsMps)
                    .withSteerInertia(PhoenixDriveConstants.kSteerInertia)
                    .withDriveInertia(PhoenixDriveConstants.kDriveInertia)
                    .withSteerFrictionVoltage(PhoenixDriveConstants.kSteerFrictionVoltage)
                    .withDriveFrictionVoltage(PhoenixDriveConstants.kDriveFrictionVoltage)
                    .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
                    .withCouplingGearRatio(PhoenixDriveConstants.kCoupleRatio);

        PhoenixDriveConstants.FrontLeft =
            PhoenixDriveConstants.ConstantCreator.createModuleConstants(
                            PhoenixDriveConstants.kFrontLeftSteerMotorId,
                            PhoenixDriveConstants.kFrontLeftDriveMotorId,
                            PhoenixDriveConstants.kFrontLeftEncoderId,
                            PhoenixDriveConstants.kFrontLeftEncoderOffset,
                            Units.inchesToMeters(PhoenixDriveConstants.kFrontLeftXPosInches),
                            Units.inchesToMeters(PhoenixDriveConstants.kFrontLeftYPosInches),
                            PhoenixDriveConstants.kInvertLeftSide)
                    .withSteerMotorInverted(PhoenixDriveConstants.kFrontLeftSteerInvert);

        PhoenixDriveConstants.FrontRight =
        PhoenixDriveConstants.ConstantCreator.createModuleConstants(
                        PhoenixDriveConstants.kFrontRightSteerMotorId,
                        PhoenixDriveConstants.kFrontRightDriveMotorId,
                        PhoenixDriveConstants.kFrontRightEncoderId,
                        PhoenixDriveConstants.kFrontRightEncoderOffset,
                        Units.inchesToMeters(PhoenixDriveConstants.kFrontRightXPosInches),
                        Units.inchesToMeters(PhoenixDriveConstants.kFrontRightYPosInches),
                        PhoenixDriveConstants.kInvertRightSide)
                .withSteerMotorInverted(PhoenixDriveConstants.kFrontRightSteerInvert);

        PhoenixDriveConstants.BackLeft =
        PhoenixDriveConstants.ConstantCreator.createModuleConstants(
                        PhoenixDriveConstants.kBackLeftSteerMotorId,
                        PhoenixDriveConstants.kBackLeftDriveMotorId,
                        PhoenixDriveConstants.kBackLeftEncoderId,
                        PhoenixDriveConstants.kBackLeftEncoderOffset,
                        Units.inchesToMeters(PhoenixDriveConstants.kBackLeftXPosInches),
                        Units.inchesToMeters(PhoenixDriveConstants.kBackLeftYPosInches),
                        PhoenixDriveConstants.kInvertLeftSide)
                .withSteerMotorInverted(PhoenixDriveConstants.kBackLeftSteerInvert)
                .withDriveMotorInverted(PhoenixDriveConstants.kBackLeftDriveInvert);

        PhoenixDriveConstants.BackRight =
        PhoenixDriveConstants.ConstantCreator.createModuleConstants(
                        PhoenixDriveConstants.kBackRightSteerMotorId,
                        PhoenixDriveConstants.kBackRightDriveMotorId,
                        PhoenixDriveConstants.kBackRightEncoderId,
                        PhoenixDriveConstants.kBackRightEncoderOffset,
                        Units.inchesToMeters(PhoenixDriveConstants.kBackRightXPosInches),
                        Units.inchesToMeters(PhoenixDriveConstants.kBackRightYPosInches),
                        PhoenixDriveConstants.kInvertRightSide)
                .withSteerMotorInverted(PhoenixDriveConstants.kBackRightSteerInvert);

        PhoenixDriveConstants.kinematics =
            new SwerveDriveKinematics(
                    new Translation2d(PhoenixDriveConstants.FrontLeft.LocationX, PhoenixDriveConstants.FrontLeft.LocationY),
                    new Translation2d(PhoenixDriveConstants.FrontLeft.LocationX, PhoenixDriveConstants.FrontRight.LocationY),
                    new Translation2d(PhoenixDriveConstants.BackLeft.LocationX, PhoenixDriveConstants.BackLeft.LocationY),
                    new Translation2d(PhoenixDriveConstants.BackRight.LocationX, PhoenixDriveConstants.BackRight.LocationY));
    }
}
