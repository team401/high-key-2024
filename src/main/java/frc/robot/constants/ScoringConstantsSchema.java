package frc.robot.constants;

import edu.wpi.first.math.util.Units;
import java.util.HashMap;

public class ScoringConstantsSchema {
    public   double aimerkP = 100.0;
    public   double aimerkI = 2.0; // 5.0
    public   double aimerkD = 0.0;

    public   double aimerkS = 0.3; // 0.25;
    public   double aimerkG = 0.955; // 0.2;
    public   double aimerkV = 0.0;
    public   double aimerkA = 0.0;

    public   double shooterkP = 0.05;
    public   double shooterkI = 1;
    public   double shooterkD = 0.0;

    public   double shooterkS = 0.0; // TODO: Find Imperically
    public   double shooterkV = 0.005;
    public   double shooterkA = 0.0;

    public   int aimerMotorId = 9;

    public   int kickerMotorId = 5; // 1

    // TODO: REPLACE THIS WHEN THE ACTUAL SHOOTER IO IS MERGED
    public   int shooterLeftMotorId = 10;
    public   int shooterRightMotorId = 11;

    public   double shooterCurrentLimit = 120;
    public   double kickerCurrentLimit = 120;
    public   double aimerCurrentLimit = 60;

    public   int aimerEncoderId = 13;

    public   double aimerEncoderOffset = -0.087402; // Armencoder is zeroed

    public   double aimerEncoderToMechanismRatio = 1.0;
    public   double aimerRotorToSensorRatio = 90.0;

    public   double kickerIntakeVolts = 2.0;

    // public   double aimPositionTolerance = 0.003;

    public   double aimerAcceleration = 3.5;
    public   double aimerCruiseVelocity = 0.8;

    public   double shooterLowerVelocityMarginRPM = 50;
    public   double shooterUpperVelocityMarginRPM = 150;
    public   double aimAngleMarginRotations = Units.degreesToRotations(1);
    public   double aimAngleVelocityMargin = 2.0;

    public   double minDistanceAlignmentNeeded = 1.3; // TODO: Tune this value

    public   double intakeAngleToleranceRotations = 0.05; // Todo: tune this value

    public   double aimerAmpPositionRot = 0.145;

    public   double aimMaxAngleRotations = 0.361328 - 0.184570;
    public   double aimMinAngleRotations = -0.212285;
    public   double aimLockVoltage = -0.5;

    public   double aimAngleTolerance = 0.015;

    public   double ampAimerAngleRotations = 0.14;

    public   double maxAimIntake = 0.0;
    public   double minAimIntake = 0.0;

    public   double shooterOffsetAdjustment = 0.6;

    public   double maxElevatorPosition = 0.45;
    public   double maxAimAngleElevatorLimit = Math.PI / 2;

    public double[] aimerDistance;
    public double[] aimerPosition;

    public   double passLocationRot = -0.14;
    public   double passAngleToleranceRot = 0.005;
    public   double passShooterRpm = 2800.0;

    public   double aimerOffset = 0.0;

    public double[] shooterDistance;
    public double[] shooterRPM;

    // NOTE - This should be monotonically increasing
    // Key - Distance in meters
    // Value - Shooter RPM
    public  HashMap<Double, Double> getShooterMap() {
        HashMap<Double, Double> map = new HashMap<Double, Double>();
        map.put(1.284, 3500.0);
        map.put(1.7, 4000.0);

        return map;
    }

    public double[] timeToGoalDistance;
    public double[] timeToGoal;

    // NOTE - This should be monotonically increasing
    // Key - Distance in meters
    // Value - Time in seconds
    public  HashMap<Double, Double> timeToGoalMap() {
        HashMap<Double, Double> map = new HashMap<Double, Double>();
        map.put(0.0, 0.15);
        map.put(1.3, 0.15);
        map.put(1.4, 0.16);
        // map.put(1.3, 0.15);

        return map;
    }

    public double[] aimerToleranceDistance;
    public double[] aimerTolerance;

    // NOTE - This should be monotonically increasing
    // Key - Distance to goal in meters
    // Value - Aimer angle tolerance in rotations
    public  HashMap<Double, Double> aimerToleranceTable() {
        HashMap<Double, Double> map = new HashMap<Double, Double>();
        map.put(0.0, 0.003); // Todo: tune this value after conversion from radians to rotations

        return map;
    }

    public HashMap<Double, Double> aimerMap;
    public HashMap<Double, Double> shooterMap;
    public HashMap<Double, Double> timeToGoalMap;
    public HashMap<Double, Double> aimerToleranceMap;
}
