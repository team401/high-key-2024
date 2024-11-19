package frc.robot.constants;

import edu.wpi.first.math.util.Units;
import java.util.HashMap;

public class ScoringConstants {
    public static final double aimerkP = 100.0;
    public static final double aimerkI = 2.0; // 5.0
    public static final double aimerkD = 0.0;

    public static final double aimerkS = 0.3; // 0.25;
    public static final double aimerkG = 0.955; // 0.2;
    public static final double aimerkV = 0.0;
    public static final double aimerkA = 0.0;

    public static final double shooterkP = 0.05;
    public static final double shooterkI = 1;
    public static final double shooterkD = 0.0;

    public static final double shooterkS = 0.0; // TODO: Find Imperically
    public static final double shooterkV = 0.005;
    public static final double shooterkA = 0.0;

    public static final int aimerMotorId = 9;

    public static final int kickerMotorId = 5; // 1

    // TODO: REPLACE THIS WHEN THE ACTUAL SHOOTER IO IS MERGED
    public static final int shooterLeftMotorId = 10;
    public static final int shooterRightMotorId = 11;

    public static final double shooterCurrentLimit = 120;
    public static final double kickerCurrentLimit = 120;
    public static final double aimerCurrentLimit = 60;

    public static final int aimerEncoderId = 13;

    public static final double aimerEncoderOffset = -0.087402; // Armencoder is zeroed

    public static final double aimerEncoderToMechanismRatio = 1.0;
    public static final double aimerRotorToSensorRatio = 90.0;

    public static final double kickerIntakeVolts = 2.0;

    // public static final double aimPositionTolerance = 0.003;

    public static final double aimerAcceleration = 3.5;
    public static final double aimerCruiseVelocity = 0.8;

    public static final double shooterLowerVelocityMarginRPM = 50;
    public static final double shooterUpperVelocityMarginRPM = 150;
    public static final double aimAngleMarginRotations = Units.degreesToRotations(1);
    public static final double aimAngleVelocityMargin = 2.0;

    public static final double minDistanceAlignmentNeeded = 1.3; // TODO: Tune this value

    public static final double intakeAngleToleranceRotations = 0.05; // Todo: tune this value

    public static final double aimerAmpPositionRot = 0.145;

    public static final double aimMaxAngleRotations = 0.361328 - 0.184570;
    public static final double aimMinAngleRotations = -0.212285;
    public static final double aimLockVoltage = -0.5;

    public static final double aimAngleTolerance = 0.015;

    public static final double ampAimerAngleRotations = 0.14;

    public static final double maxAimIntake = 0.0;
    public static final double minAimIntake = 0.0;

    public static final double shooterOffsetAdjustment = 0.6;

    public static final double maxElevatorPosition = 0.45;
    public static final double maxAimAngleElevatorLimit = Math.PI / 2;

    // NOTE - This should be monotonically increasing
    // Key - Distance in meters
    // Value - Aimer angle in rotations
    public static HashMap<Double, Double> getAimerMap() {
        HashMap<Double, Double> map = new HashMap<Double, Double>();
        // map.put(1.284, -0.1);
        map.put(1.428, -0.1);
        // map.put(1.7, -0.13);
        // map.put(2.30, -0.15);
        // map.put(3.32, -0.175);
        // map.put(4.23, -0.195);
        // map.put(5.811, -0.2);

        return map;
    }


    public static final double passLocationRot = -0.14;
    public static final double passAngleToleranceRot = 0.005;
    public static final double passShooterRpm = 2800.0;


    public static final double aimerStaticOffset = 0.0;

    // NOTE - This should be monotonically increasing
    // Key - Distance in meters
    // Value - Shooter RPM
    public static HashMap<Double, Double> getShooterMap() {
        HashMap<Double, Double> map = new HashMap<Double, Double>();
        map.put(1.284, 3500.0);
        map.put(1.7, 4000.0);

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
    // Key - Distance to goal in meters
    // Value - Aimer angle tolerance in rotations
    public static HashMap<Double, Double> aimerToleranceTable() {
        HashMap<Double, Double> map = new HashMap<Double, Double>();
        map.put(0.0, 0.003); // Todo: tune this value after conversion from radians to rotations

        return map;
    }
}
