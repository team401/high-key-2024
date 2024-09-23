package frc.robot.constants;

import coppercore.parameter_tools.JSONExclude;
import coppercore.parameter_tools.JSONSync;

public final class ConversionConstants {

    @JSONExclude
    public static final JSONSync<ConversionConstants> synced =
            new JSONSync<ConversionConstants>(
                    new ConversionConstants(),
                    "filePath",
                    new JSONSync.JSONSyncConfigBuilder().build());

    public final double kRadiansPerSecondToRPM = 60.0 / (2.0 * Math.PI);
    public final double kRPMToRadiansPerSecond = 1.0 / kRadiansPerSecondToRPM;

    public final double kSecondsToMinutes = 1.0 / 60.0;
    public final double kMinutesToSeconds = 60.0;

    public final double kDegreesToRadians = Math.PI / 180.0;
    public final double kRadiansToDegrees = 180.0 / Math.PI;
}
