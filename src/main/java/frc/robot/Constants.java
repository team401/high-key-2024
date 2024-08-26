package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.util.Units;

public class Constants {

    public static final double loopTime = 0.02;

    public static final class ConversionConstants {
        public static final double kRadiansPerSecondToRPM = 60.0 / (2.0 * Math.PI);
        public static final double kRPMToRadiansPerSecond = 1.0 / kRadiansPerSecondToRPM;

        public static final double kSecondsToMinutes = 1.0 / 60.0;
        public static final double kMinutesToSeconds = 60.0;

        public static final double kDegreesToRadians = Math.PI / 180.0;
        public static final double kRadiansToDegrees = 180.0 / Math.PI;
    }


    public static final class SensorConstants {

        //TODO: Find real values
        public static final int indexerSensorPort = 0;
    }
    
public static final class ScoringConstants {
        public static final double aimerkP = 17.0;
        public static final double aimerkI = 10.0; // 5.0
        public static final double aimerkD = 0.0;

        public static final double aimerkS = 0.265;
        public static final double aimerkG = 0.1;
        public static final double aimerkV = 1.51;
        public static final double aimerkA = 0.01;

        public static final double shooterkP = 0.05;
        public static final double shooterkI = 0.2;
        public static final double shooterkD = 0.0;

        public static final double shooterkS = 0.0; // TODO: Find Imperically
        public static final double shooterkV = 0.0095;
        public static final double shooterkA = 0.0;

        public static final double hoodkP = 0.05;
        public static final double hoodkI = 0.0;
        public static final double hoodkD = 0.0;

        public static final double hoodkS = 0.14; // 0.14
        public static final double hoodkG = 0.41; // 0.41
        public static final double hoodkV = 0.0;

        public static final double hoodPositionTolerance = 0.01;

        public static final double hoodEncoderToRad = 1.3 * (15.0 / 38.0) * (2.0 * Math.PI);

        public static final int aimLeftMotorId = 16;
        public static final int aimRightMotorId = 15;

        public static final int shooterLeftMotorId = 11;
        public static final int shooterRightMotorId = 12;

        public static final int kickerMotorId = 13;

        public static final int hoodId = 17;

        public static final int aimEncoderPort = 0;
        public static final double aimerEncoderOffset = 1.75 - 0.01; // 0.027

        public static final double kickerIntakeVolts = 2.0;

        public static final double aimPositionTolerance = 0.017;

        public static final double aimAcceleration = 4.5; // TODO: 15.0
        public static final double aimCruiseVelocity = 7.0; // TODO: 15.0

        public static final double shooterLowerVelocityMarginRPM = 50;
        public static final double shooterUpperVelocityMarginRPM = 150;
        public static final double aimAngleMarginRadians = Units.degreesToRadians(1);
        public static final double aimAngleVelocityMargin = 2.0; // Units.degreesToRadians(5);
        public static final double hoodAngleMarginRadians = Units.degreesToRadians(5);

        public static final double intakeAngleToleranceRadians = 0.2;
        // Math.PI / 2 - Units.degreesToRadians(40);

        public static final double shooterAmpVelocityRPM = 2000;

        public static final double hoodHomeAmps = 40.0; // TODO: Find this
        public static final double hoodHomeAngleRad = Math.PI - 0.23;

        public static final double aimMaxAngleRadians = 1.65; // Math.PI / 2
        public static final double aimMinAngleRadians = -0.03;

        public static final double maxAimIntake = 0.0;
        public static final double minAimIntake = 0.0;

        public static final double shooterOffsetAdjustment = 0.6;

        public static final double maxElevatorPosition = 0.45;
        public static final double maxAimAngleElevatorLimit = Math.PI / 2;

        public static final double hoodMaxVelocity = 0.5;
        public static final double hoodMaxAcceleration = 0.5;

        // NOTE - This should be monotonically increasing
        // Key - Distance in meters
        // Value - Aimer angle in radians
        public static HashMap<Double, Double> getAimerMap() {
            HashMap<Double, Double> map = new HashMap<Double, Double>();
            map.put(0.0, 0.8);
            map.put(1.45, 0.8);
            map.put(1.98, 0.62);
            map.put(2.41, 0.53);
            map.put(3.02, 0.45);
            map.put(3.22, 0.45);
            map.put(3.9, 0.36);
            map.put(4.55, 0.35);
            map.put(4.95, 0.32);
            map.put(5.15, 0.295);
            map.put(5.35, 0.295);
            map.put(5.5, 0.295);
            map.put(5.64, 0.29);
            // map.put(5.82, 0.275);
            map.put(6.0, 0.29);

            return map;
        }

        public static final double aimerStaticOffset = 0.01;

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

}
