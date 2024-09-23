package frc.robot.constants;

import coppercore.parameter_tools.JSONExclude;
import coppercore.parameter_tools.JSONSync;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class FieldConstants {

    @JSONExclude
    public static final JSONSync<FieldConstants> synced =
            new JSONSync<FieldConstants>(
                    new FieldConstants(), "filePath", new JSONSync.JSONSyncConfigBuilder().build());

    public final double lengthM = 16.451;
    public final double widthM = 8.211;

    public final double midfieldLowThresholdM = 5.87;
    public final double midfieldHighThresholdM = 10.72;

    @JSONExclude public final Rotation2d ampHeading = new Rotation2d(-Math.PI / 2);

    @JSONExclude public final Rotation2d blueUpHeading = Rotation2d.fromRadians(0.0);
    @JSONExclude public final Rotation2d blueDownHeading = Rotation2d.fromRadians(Math.PI);

    @JSONExclude public final Rotation2d blueLeftHeading = Rotation2d.fromRadians(Math.PI / 2.0);

    @JSONExclude public final Rotation2d blueRightHeading = Rotation2d.fromRadians(-Math.PI / 2.0);

    @JSONExclude public final Rotation2d redUpHeading = Rotation2d.fromRadians(Math.PI);
    @JSONExclude public final Rotation2d redDownHeading = Rotation2d.fromRadians(0.0);

    @JSONExclude public final Rotation2d redLeftHeading = Rotation2d.fromRadians(-Math.PI / 2.0);

    @JSONExclude public final Rotation2d redRightHeading = Rotation2d.fromRadians(Math.PI / 2.0);

    @JSONExclude
    public final Rotation2d redSourceHeading = new Rotation2d(Math.PI * 4 / 3); // 60 degrees

    @JSONExclude
    public final Rotation2d blueSourceHeading = new Rotation2d(Math.PI * 5 / 3); // 120 degrees

    @JSONExclude
    public final Translation2d fieldToRedSpeaker =
            new Translation2d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42));

    @JSONExclude
    public final Translation2d fieldToBlueSpeaker =
            new Translation2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42));

    @JSONExclude
    public final Pose2d robotAgainstBlueSpeaker =
            new Pose2d(1.39, 5.56, Rotation2d.fromDegrees(180));

    @JSONExclude
    public final Pose2d robotAgainstRedSpeaker = new Pose2d(15.19, 5.56, Rotation2d.fromDegrees(0));

    @JSONExclude
    public final Pose2d robotAgainstBlueSpeakerRight =
            new Pose2d(0.7, 4.38, Rotation2d.fromDegrees(120));

    @JSONExclude
    public final Pose2d robotAgainstRedSpeakerRight =
            new Pose2d(15.83, 6.73, Rotation2d.fromDegrees(-60));

    @JSONExclude
    public final Pose2d robotAgainstBlueSpeakerLeft =
            new Pose2d(0.7, 6.73, Rotation2d.fromDegrees(-120));

    @JSONExclude
    public final Pose2d robotAgainstRedSpeakerLeft =
            new Pose2d(15.83, 4.38, Rotation2d.fromDegrees(60));

    @JSONExclude
    public final Pose2d robotAgainstBluePodium =
            new Pose2d(2.57, 4.09, Rotation2d.fromDegrees(180));

    @JSONExclude
    public final Pose2d robotAgainstRedPodium = new Pose2d(13.93, 4.09, Rotation2d.fromDegrees(0));

    @JSONExclude
    public final Pose2d robotAgainstBlueAmpZone =
            new Pose2d(2.85, 7.68, Rotation2d.fromDegrees(-90));

    @JSONExclude
    public final Pose2d robotAgainstRedAmpZone =
            new Pose2d(13.74, 7.68, Rotation2d.fromDegrees(-90));
}
