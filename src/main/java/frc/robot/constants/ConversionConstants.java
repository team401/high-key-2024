package frc.robot.constants;

public  class ConversionConstants {
    public   double kRadiansPerSecondToRPM = 60.0 / (2.0 * Math.PI);
    public   double kRPMToRadiansPerSecond = 1.0 / kRadiansPerSecondToRPM;

    public   double kSecondsToMinutes = 1.0 / 60.0;
    public   double kMinutesToSeconds = 60.0;

    public   double kDegreesToRadians = Math.PI / 180.0;
    public   double kRadiansToDegrees = 180.0 / Math.PI;
}
