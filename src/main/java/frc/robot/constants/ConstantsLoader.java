package frc.robot.constants;

import java.util.HashMap;

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
    public static ScoringConstantsSchema ScoringConstants;

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

    public static void loadScoringConstants() {
        JSONSync<ScoringConstantsSchema> synced =
                new JSONSync<ScoringConstantsSchema>(
                        new ScoringConstantsSchema(),
                        "./DriverConstants.json",
                        new JSONSync.JSONSyncConfigBuilder().build());
        synced.loadData();
        ScoringConstants = synced.getObject();
        ScoringConstants.aimerMap = new HashMap<Double, Double>();
        ScoringConstants.shooterMap = new HashMap<Double, Double>();
        ScoringConstants.timeToGoalMap = new HashMap<Double, Double>();
        ScoringConstants.aimerToleranceMap = new HashMap<Double, Double>();

        for (int i = 0; i < ScoringConstants.aimerDistance.length; i++) {
            ScoringConstants.aimerMap.put(ScoringConstants.aimerDistance[i], ScoringConstants.aimerPosition[i]);
        }

        for (int i = 0; i < ScoringConstants.shooterDistance.length; i++) {
            ScoringConstants.shooterMap.put(ScoringConstants.shooterDistance[i], ScoringConstants.shooterRPM[i]);
        }

        for (int i = 0; i < ScoringConstants.timeToGoalDistance.length; i++) {
            ScoringConstants.timeToGoalMap.put(ScoringConstants.timeToGoalDistance[i], ScoringConstants.timeToGoal[i]);
        }

        for (int i = 0; i < ScoringConstants.aimerToleranceDistance.length; i++) {
            ScoringConstants.aimerToleranceMap.put(ScoringConstants.aimerToleranceDistance[i], ScoringConstants.aimerTolerance[i]);
        }
    }
}
