package frc.robot.constants;

import coppercore.parameter_tools.JSONExclude;
import coppercore.parameter_tools.JSONSync;

public final class FeatureFlags {

    @JSONExclude
    public static final JSONSync<FeatureFlags> synced =
            new JSONSync<FeatureFlags>(
                    new FeatureFlags(), "filePath", new JSONSync.JSONSyncConfigBuilder().build());

    public final boolean simulateDrive = true;
    public final boolean runDrive = true;
    public final boolean simulateVision = true;
    public final boolean runVision = true;
    public final boolean runLocalizer = true;
    public final boolean simulateIntake = true;
    public final boolean runIntake = true;
    public final boolean simulateScoring = true;
    public final boolean runScoring = true;
}
