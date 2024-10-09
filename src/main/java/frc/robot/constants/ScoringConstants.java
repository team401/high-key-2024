package frc.robot.constants;

import edu.wpi.first.math.util.Units;
import java.util.HashMap;

public class ScoringConstants {
    public static final double aimerkP = 23.0;
    public static final double aimerkI = 4.0; // 5.0
    public static final double aimerkD = 0.0;

    public static final double aimerkS = 0.0;
    public static final double aimerkG = 0.2;
    public static final double aimerkV = 0.0;
    public static final double aimerkA = 0.0;

    public static final double shooterkP = 0.05;
    public static final double shooterkI = 0.2;
    public static final double shooterkD = 0.0;

    public static final double shooterkS = 0.0; // TODO: Find Imperically
    public static final double shooterkV = 0.0095;
    public static final double shooterkA = 0.0;

    public static final int aimerMotorId = 9;

    public static final int kickerMotorId = 13;

    // TODO: REPLACE THIS WHEN THE ACTUAL SHOOTER IO IS MERGED
    public static final int shooterLeftMotorId = 10;
    public static final int shooterRightMotorId = 11;

    public static final double shooterCurrentLimit = 120;
    public static final double kickerCurrentLimit = 120;
    public static final double aimerCurrentLimit = 60;

    public static final int aimerEncoderId = 13;
    public static final double aimerEncoderOffset = 0.184570;

    public static final double aimerEncoderToMechanismRatio = 1.0;
    public static final double aimerRotorToSensorRatio = 90.0;

    public static final double kickerIntakeVolts = 2.0;

    public static final double aimPositionTolerance = 0.017;

    // These values have been reduced for tuning because we can't set a voltage limit on the motors
    // anymore
    // public static final double aimerAcceleration = 4.5; // TODO: 15.0
    // public static final double aimerCruiseVelocity = 7.0; // TODO: 15.0
    public static final double aimerAcceleration = 0.7;
    public static final double aimerCruiseVelocity = 0.4;

    public static final double shooterLowerVelocityMarginRPM = 50;
    public static final double shooterUpperVelocityMarginRPM = 150;
    public static final double aimAngleMarginRadians = Units.degreesToRadians(1);
    public static final double aimAngleVelocityMargin = 2.0; // Units.degreesToRadians(5);

    public static final double intakeAngleToleranceRadians = 0.2;
    // Math.PI / 2 - Units.degreesToRadians(40);

    public static final double shooterAmpVelocityRPM = 2000;

    public static final double aimMaxAngleRadians = 0.361328 - 0.184570; // Math.PI / 2
    public static final double aimMinAngleRadians = -0.037598 - 0.184570;
    public static final double aimAngleTolerance = 0.015;

    public static final double maxAimIntake = 0.0;
    public static final double minAimIntake = 0.0;

    public static final double shooterOffsetAdjustment = 0.6;

    public static final double maxElevatorPosition = 0.45;
    public static final double maxAimAngleElevatorLimit = Math.PI / 2;

    // NOTE - This should be monotonically increasing
    // Key - Distance in meters
    // Value - Aimer angle in radians
    public static HashMap<Double, Double> getAimerMap() {
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

    public static final double aimerStaticOffset = 0.0;

    // NOTE - This should be monotonically increasing
    // Key - Distance in meters
    // Value - Shooter RPM
    public static HashMap<Double, Double> getShooterMap() {
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
    public static HashMap<Double, Double> timeToGoalMap() {
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
    public static HashMap<Double, Double> timeToPutAimDownMap() { // TODO: Find this
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
    public static HashMap<Double, Double> aimerAvoidElevatorTable() {
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
    public static HashMap<Double, Double> aimerToleranceTable() {
        HashMap<Double, Double> map = new HashMap<Double, Double>();
        map.put(0.0, 0.1);
        map.put(2.0, 0.1);
        map.put(10.0, 0.015);

        return map;
    }
}
