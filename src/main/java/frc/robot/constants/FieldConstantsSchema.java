package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public  class FieldConstantsSchema {
    public   double lengthM = 16.451;
    public   double widthM = 8.211;

    public   double midfieldLowThresholdM = 5.87;
    public   double midfieldHighThresholdM = 10.72;

    public   Rotation2d ampHeading = new Rotation2d(Math.PI / 2);

    public   Rotation2d blueUpHeading = Rotation2d.fromRadians(0.0);
    public   Rotation2d blueDownHeading = Rotation2d.fromRadians(Math.PI);
    public   Rotation2d blueLeftHeading = Rotation2d.fromRadians(Math.PI / 2.0);
    public   Rotation2d blueRightHeading = Rotation2d.fromRadians(-Math.PI / 2.0);

    public   Rotation2d redUpHeading = Rotation2d.fromRadians(Math.PI);
    public   Rotation2d redDownHeading = Rotation2d.fromRadians(0.0);
    public   Rotation2d redLeftHeading = Rotation2d.fromRadians(-Math.PI / 2.0);
    public   Rotation2d redRightHeading = Rotation2d.fromRadians(Math.PI / 2.0);

    public   Rotation2d redSourceHeading = new Rotation2d(Math.PI * 4 / 3); // 60 degrees
    public   Rotation2d blueSourceHeading =
            new Rotation2d(Math.PI * 5 / 3); // 120 degrees

    public   Translation2d fieldToRedSpeaker =
            new Translation2d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42));

    public   Translation2d fieldToBlueSpeaker =
            new Translation2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42));

    public   Translation2d fieldToBluePass =
            new Translation2d(Units.inchesToMeters(25), Units.inchesToMeters(230));

    public   Translation2d fieldToRedPass =
            new Translation2d(Units.inchesToMeters(625), Units.inchesToMeters(230));

    public   Pose2d robotAgainstBlueSpeaker =
            new Pose2d(1.39, 5.56, Rotation2d.fromDegrees(180));

    public   Pose2d robotAgainstRedSpeaker =
            new Pose2d(15.19, 5.56, Rotation2d.fromDegrees(0));

    public   Pose2d robotAgainstBlueSpeakerRight =
            new Pose2d(0.7, 4.38, Rotation2d.fromDegrees(120));

    public   Pose2d robotAgainstRedSpeakerRight =
            new Pose2d(15.83, 6.73, Rotation2d.fromDegrees(-60));

    public   Pose2d robotAgainstBlueSpeakerLeft =
            new Pose2d(0.7, 6.73, Rotation2d.fromDegrees(-120));

    public   Pose2d robotAgainstRedSpeakerLeft =
            new Pose2d(15.83, 4.38, Rotation2d.fromDegrees(60));

    public   Pose2d robotAgainstBluePodium =
            new Pose2d(2.57, 4.09, Rotation2d.fromDegrees(180));

    public   Pose2d robotAgainstRedPodium =
            new Pose2d(13.93, 4.09, Rotation2d.fromDegrees(0));

    public   Pose2d robotAgainstBlueAmpZone =
            new Pose2d(2.85, 7.68, Rotation2d.fromDegrees(-90));

    public   Pose2d robotAgainstRedAmpZone =
            new Pose2d(13.74, 7.68, Rotation2d.fromDegrees(-90));
}
