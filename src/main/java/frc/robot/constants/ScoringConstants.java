package frc.robot.constants;

import coppercore.parameter_tools.JSONExclude;
import coppercore.parameter_tools.JSONSync;
import edu.wpi.first.math.util.Units;
import java.util.HashMap;

public class ScoringConstants {

    @JSONExclude
    public static final JSONSync<ScoringConstants> synced =
            new JSONSync<ScoringConstants>(
                    new ScoringConstants(),
                    "filePath",
                    new JSONSync.JSONSyncConfigBuilder().build());

    public final double aimerkP = 17.0;
    public final double aimerkI = 10.0; // 5.0
    public final double aimerkD = 0.0;

    public final double aimerkS = 0.265;
    public final double aimerkG = 0.1;
    public final double aimerkV = 1.51;
    public final double aimerkA = 0.01;

    public final double shooterkP = 0.05;
    public final double shooterkI = 0.2;
    public final double shooterkD = 0.0;

    public final double shooterkS = 0.0; // TODO: Find Imperically
    public final double shooterkV = 0.0095;
    public final double shooterkA = 0.0;

    public final int aimLeftMotorId = 16;
    public final int aimRightMotorId = 15;

    public final int shooterLeftMotorId = 11;
    public final int shooterRightMotorId = 12;

    public final int kickerMotorId = 13;

    public final double shooterCurrentLimit = 120;
    public final double kickerCurrentLimit = 120;
    public final double aimerCurrentLimit = 60;

    public final int aimEncoderPort = 0;
    public final double aimerEncoderOffset = 1.75 - 0.01; // 0.027

    public final double kickerIntakeVolts = 2.0;

    public final double aimPositionTolerance = 0.017;

    public final double aimAcceleration = 4.5; // TODO: 15.0
    public final double aimCruiseVelocity = 7.0; // TODO: 15.0

    public final double shooterLowerVelocityMarginRPM = 50;
    public final double shooterUpperVelocityMarginRPM = 150;
    public final double aimAngleMarginRadians = Units.degreesToRadians(1);
    public final double aimAngleVelocityMargin = 2.0; // Units.degreesToRadians(5);

    public final double intakeAngleToleranceRadians = 0.2;
    // Math.PI / 2 - Units.degreesToRadians(40);

    public final double shooterAmpVelocityRPM = 2000;

    public final double aimMaxAngleRadians = 2 * Math.PI; // Math.PI / 2
    public final double aimMinAngleRadians = Math.PI;
    public final double aimAngleTolerance = 0.015;

    public final double maxAimIntake = 0.0;
    public final double minAimIntake = 0.0;

    public final double shooterOffsetAdjustment = 0.6;

    public final double maxElevatorPosition = 0.45;
    public final double maxAimAngleElevatorLimit = Math.PI / 2;

    // NOTE - This should be monotonically increasing
    // Key - Distance in meters
    // Value - Aimer angle in radians

    public HashMap<Double, Double> getAimerMap() {
        HashMap<Double, Double> map = new HashMap<Double, Double>();
        map.put(0.0, 2 * Math.PI - 0.8);
        map.put(1.45, 2 * Math.PI - 0.8);
        map.put(1.98, 2 * Math.PI - 0.62);
        map.put(2.41, 2 * Math.PI - 0.53);
        map.put(3.02, 2 * Math.PI - 0.45);
        map.put(3.22, 2 * Math.PI - 0.45);
        map.put(3.9, 2 * Math.PI - 0.36);
        map.put(4.55, 2 * Math.PI - 0.35);
        map.put(4.95, 2 * Math.PI - 0.32);
        map.put(5.15, 2 * Math.PI - 0.295);
        map.put(5.35, 2 * Math.PI - 0.295);
        map.put(5.5, 2 * Math.PI - 0.295);
        map.put(5.64, 2 * Math.PI - 0.29);
        // map.put(5.82, 0.275);
        map.put(6.0, 2 * Math.PI - 0.29);

        return map;
    }

    public final double aimerStaticOffset = 0.01;

    // NOTE - This should be monotonically increasing
    // Key - Distance in meters
    // Value - Shooter RPM
    public HashMap<Double, Double> getShooterMap() {
        HashMap<Double, Double> map = new HashMap<Double, Double>();
        map.put(0.0, 2700.0);
        map.put(1.45, 2700.0);
        map.put(1.98, 2700.0);
        map.put(2.41, 3000.0);
        map.put(3.02, 3300.0);
        map.put(3.22, 3300.0);
        map.put(3.9, 3300.0);
        map.put(4.55, 3500.0);
        map.put(4.95, 3500.0); // 4000.0
        map.put(5.15, 3500.0);
        map.put(5.35, 3500.0);
        map.put(5.64, 3500.0);
        map.put(5.82, 3500.0);

        return map;
    }

    // NOTE - This should be monotonically increasing
    // Key - Distance in meters
    // Value - Time in seconds
    public HashMap<Double, Double> timeToGoalMap() {
        HashMap<Double, Double> map = new HashMap<Double, Double>();
        map.put(0.0, 0.15);
        map.put(1.3, 0.15);
        map.put(1.4, 0.16);
        // map.put(1.3, 0.15);

        return map;
    }

    // NOTE - This should be monotonically increasing
    // Key - Angle in radians
    // Value - Time in seconds
    public HashMap<Double, Double> timeToPutAimDownMap() { // TODO: Find this
        HashMap<Double, Double> map = new HashMap<Double, Double>();
        map.put(0.0, 0.6);
        map.put(Math.PI / 6, 0.6);
        map.put(Math.PI / 4, 0.7);
        map.put(Math.PI / 3, 0.8);
        map.put(Math.PI / 2, 1.0);

        return map;
    }

    // NOTE - This should be monotonically increasing
    // Key - Elevator position in meters
    // Value - Aimer angle in radians
    public HashMap<Double, Double> aimerAvoidElevatorTable() {
        HashMap<Double, Double> map = new HashMap<Double, Double>();
        map.put(0.0, 0.0);
        map.put(0.01, Math.PI / 8);
        map.put(0.05, Math.PI / 6);
        map.put(0.1, Math.PI / 4);
        map.put(0.2, Math.PI / 3);
        map.put(0.4, 1.37);

        return map;
    }

    // NOTE - This should be monotonically increasing
    // Key - Distance to goal in meters
    // Value - Aimer angle tolerance in radians
    public HashMap<Double, Double> aimerToleranceTable() {
        HashMap<Double, Double> map = new HashMap<Double, Double>();
        map.put(0.0, 0.1);
        map.put(2.0, 0.1);
        map.put(10.0, 0.015);

        return map;
    }
}
