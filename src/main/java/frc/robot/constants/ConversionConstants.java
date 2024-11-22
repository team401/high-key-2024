package frc.robot.constants;

public  class ConversionConstants {
    public  static double kRadiansPerSecondToRPM = 60.0 / (2.0 * Math.PI);
    public static  double kRPMToRadiansPerSecond = 1.0 / kRadiansPerSecondToRPM;

    public  static double kSecondsToMinutes = 1.0 / 60.0;
    public static  double kMinutesToSeconds = 60.0;

    public static  double kDegreesToRadians = Math.PI / 180.0;
    public  static double kRadiansToDegrees = 180.0 / Math.PI;
}
