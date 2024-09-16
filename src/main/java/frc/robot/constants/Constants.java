package frc.robot.constants;

import java.util.HashMap;

import edu.wpi.first.math.util.Units;

public class Constants {

    public static final double loopTime = 0.02;

    public static final Mode currentMode = Mode.SIM;

    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }

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

    public static final class FeatureFlags {
        public static final boolean simulateVision = true;
        public static final boolean runVision = true;
        public static final boolean runLocalizer = true;

        public static final boolean runIntake = true;
        public static final boolean runScoring = true;
        public static final boolean runEndgame = true;
        public static final boolean runDrive = true;

        public static final boolean enableLEDS = true;
    }

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
        public static final int uptakeSensorPort = 0;
    }

    
}
