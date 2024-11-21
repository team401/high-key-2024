package frc.robot.constants;

import coppercore.parameter_tools.JSONSync;

public class ConstantsLoader {
    /*
     * NOTE: Each json constants file needs a schema matching its structure
     *
     * -> that schema is then put into a new JSON Sync class,
     *      class' load method is called,
     *      and then object is retrieved for holding in corresponding static variable
     *
     * IMPORTANT: file paths are from base folder (High-Key-2024)
     */
    public static DriverConstantsSchema DriverConstants;
    public static FeatureFlagsSchema FeatureFlags;
    public static FieldConstantsSchema FieldConstants;

    public static void loadDriverConstants() {
        JSONSync<DriverConstantsSchema> synced =
                new JSONSync<DriverConstantsSchema>(
                        new DriverConstantsSchema(),
                        "./DriverConstants.json",
                        new JSONSync.JSONSyncConfigBuilder().build());
        synced.loadData();
        DriverConstants = synced.getObject();
    }

    public static void loadFeatureFlags() {
        JSONSync<FeatureFlagsSchema> synced =
                new JSONSync<FeatureFlagsSchema>(
                        new FeatureFlagsSchema(),
                        "./DriverConstants.json",
                        new JSONSync.JSONSyncConfigBuilder().build());
        synced.loadData();
        FeatureFlags = synced.getObject();
    }

    public static void loadFieldConstants() {
                JSONSync<FieldConstantsSchema> synced =
                new JSONSync<FieldConstantsSchema>(
                        new FieldConstantsSchema(),
                        "./DriverConstants.json",
                        new JSONSync.JSONSyncConfigBuilder().build());
        synced.loadData();
        FieldConstants = synced.getObject();
    }
}
