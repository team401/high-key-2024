package frc.robot.constants;

import coppercore.parameter_tools.JSONExclude;
import coppercore.parameter_tools.JSONSync;

public final class SensorConstants {

    @JSONExclude
    public static final JSONSync<SensorConstants> synced =
            new JSONSync<SensorConstants>(
                    new SensorConstants(),
                    "filePath",
                    new JSONSync.JSONSyncConfigBuilder().build());

    // TODO: Find real values
    public final int indexerSensorPort = 0;
    public final int uptakeSensorPort = 0;
}
