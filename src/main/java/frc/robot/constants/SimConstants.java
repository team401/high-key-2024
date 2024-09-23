package frc.robot.constants;

import coppercore.parameter_tools.JSONExclude;
import coppercore.parameter_tools.JSONSync;

public final class SimConstants {

    @JSONExclude
    public static final JSONSync<SimConstants> synced =
            new JSONSync<SimConstants>(
                    new SimConstants(), "filePath", new JSONSync.JSONSyncConfigBuilder().build());

    public final double loopTime = 0.02;
}
