package frc.robot.constants;

import edu.wpi.first.math.util.Units;
import java.util.HashMap;

public class ScoringConstantsSchema {
    public static  double aimerkP = 100.0;
    public  static double aimerkI = 2.0; // 5.0
    public static  double aimerkD = 0.0;

    public static  double aimerkS = 0.3; // 0.25;
    public static  double aimerkG = 0.955; // 0.2;
    public static  double aimerkV = 0.0;
    public static  double aimerkA = 0.0;

    public static  double shooterkP = 0.05;
    public static  double shooterkI = 1;
    public static  double shooterkD = 0.0;

    public static  double shooterkS = 0.0; // TODO: Find Imperically
    public static  double shooterkV = 0.005;
    public static  double shooterkA = 0.0;

    public static  int aimerMotorId = 9;

    public static  int kickerMotorId = 5; // 1

    // TODO: REPLACE THIS WHEN THE ACTUAL SHOOTER IO IS MERGED
    public static  int shooterLeftMotorId = 10;
    public static  int shooterRightMotorId = 11;

    public static  double shooterCurrentLimit = 120;
    public static  double kickerCurrentLimit = 120;
    public static  double aimerCurrentLimit = 60;

    public static  int aimerEncoderId = 13;

    public static  double aimerEncoderOffset = -0.087402; // Armencoder is zeroed

    public static  double aimerEncoderToMechanismRatio = 1.0;
    public static  double aimerRotorToSensorRatio = 90.0;

    public static  double kickerIntakeVolts = 2.0;

    // public   double aimPositionTolerance = 0.003;

    public static  double aimerAcceleration = 3.5;
    public static  double aimerCruiseVelocity = 0.8;

    public static   double shooterLowerVelocityMarginRPM = 50;
    public  static double shooterUpperVelocityMarginRPM = 150;
    public static  double aimAngleMarginRotations = Units.degreesToRotations(1);
    public static  double aimAngleVelocityMargin = 2.0;

    public static  double minDistanceAlignmentNeeded = 1.3; // TODO: Tune this value

    public static  double intakeAngleToleranceRotations = 0.05; // Todo: tune this value

    public static  double aimerAmpPositionRot = 0.145;

    public static  double aimMaxAngleRotations = 0.361328 - 0.184570;
    public static  double aimMinAngleRotations = -0.212285;
    public static  double aimLockVoltage = -0.5;

    public static  double aimAngleTolerance = 0.015;

    public static  double ampAimerAngleRotations = 0.14;

    public static  double maxAimIntake = 0.0;
    public  static double minAimIntake = 0.0;

    public static  double shooterOffsetAdjustment = 0.6;

    public static  double maxElevatorPosition = 0.45;
    public static  double maxAimAngleElevatorLimit = Math.PI / 2;

    public static double[] aimerDistance;
    public static double[] aimerPosition;

    public static  double passLocationRot = -0.14;
    public static  double passAngleToleranceRot = 0.005;
    public static  double passShooterRpm = 2800.0;

    public static  double aimerOffset = 0.0;

    public static double[] shooterDistance;
    public static double[] shooterRPM;

    public static double[] timeToGoalDistance;
    public static double[] timeToGoal;

    public static double[] aimerToleranceDistance;
    public static double[] aimerTolerance;

    public static HashMap<Double, Double> aimerMap;
    public static HashMap<Double, Double> shooterMap;
    public static HashMap<Double, Double> timeToGoalMap;
    public static HashMap<Double, Double> aimerToleranceMap;
}
