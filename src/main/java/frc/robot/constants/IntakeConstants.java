package frc.robot.constants;

import coppercore.parameter_tools.JSONExclude;
import coppercore.parameter_tools.JSONSync;

public class IntakeConstants {

    @JSONExclude
    public static final JSONSync<IntakeConstants> synced =
            new JSONSync<IntakeConstants>(
                    new IntakeConstants(),
                    "filePath",
                    new JSONSync.JSONSyncConfigBuilder().build());

    public final int leftIntakeMotorID = 9;
    public final int rightIntakeMotorID = 10;
    public final int indexTwoMotorID = 14;

    public final double intakePower = 12.0;
    public final double beltPower = 12.0;
}
