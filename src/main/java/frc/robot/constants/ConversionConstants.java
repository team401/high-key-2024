package frc.robot.constants;

public final class ConversionConstants {
    public static final double kRadiansPerSecondToRPM = 60.0 / (2.0 * Math.PI);
    public static final double kRPMToRadiansPerSecond = 1.0 / kRadiansPerSecondToRPM;

    public static final double kSecondsToMinutes = 1.0 / 60.0;
    public static final double kMinutesToSeconds = 60.0;

    public static final double kDegreesToRadians = Math.PI / 180.0;
    public static final double kRadiansToDegrees = 180.0 / Math.PI;
}
