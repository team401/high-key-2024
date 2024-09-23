package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.FieldConstants;
import org.littletonrobotics.junction.Logger;

public class AllianceUtil {

    public static Translation2d getFieldToSpeaker() {
        if (!DriverStation.getAlliance().isEmpty()) {
            switch (DriverStation.getAlliance().get()) {
                case Blue:
                    Logger.recordOutput(
                            "Field/speaker", FieldConstants.synced.getObject().fieldToBlueSpeaker);
                    return FieldConstants.synced.getObject().fieldToBlueSpeaker;
                case Red:
                    Logger.recordOutput(
                            "Field/speaker", FieldConstants.synced.getObject().fieldToRedSpeaker);
                    return FieldConstants.synced.getObject().fieldToRedSpeaker;
            }
        }
        return FieldConstants.synced.getObject().fieldToRedSpeaker;
    }

    public static Rotation2d getAmpHeading() {
        Logger.recordOutput("Field/amp", FieldConstants.synced.getObject().ampHeading);
        return FieldConstants.synced.getObject().ampHeading;
    }

    public static Pose2d getPoseAgainstSpeaker() {
        if (!DriverStation.getAlliance().isEmpty()) {
            switch (DriverStation.getAlliance().get()) {
                case Blue:
                    return FieldConstants.synced.getObject().robotAgainstBlueSpeaker;
                case Red:
                    return FieldConstants.synced.getObject().robotAgainstRedSpeaker;
            }
        }
        return FieldConstants.synced.getObject().robotAgainstRedSpeaker;
    }

    public static Pose2d getPoseAgainstSpeakerLeft() {
        if (!DriverStation.getAlliance().isEmpty()) {
            switch (DriverStation.getAlliance().get()) {
                case Blue:
                    return FieldConstants.synced.getObject().robotAgainstBlueSpeakerLeft;
                case Red:
                    return FieldConstants.synced.getObject().robotAgainstRedSpeakerLeft;
            }
        }
        return FieldConstants.synced.getObject().robotAgainstRedSpeakerLeft;
    }

    public static Pose2d getPoseAgainstSpeakerRight() {
        if (!DriverStation.getAlliance().isEmpty()) {
            switch (DriverStation.getAlliance().get()) {
                case Blue:
                    return FieldConstants.synced.getObject().robotAgainstBlueSpeakerRight;
                case Red:
                    return FieldConstants.synced.getObject().robotAgainstRedSpeakerRight;
            }
        }
        return FieldConstants.synced.getObject().robotAgainstRedSpeakerRight;
    }

    public static Pose2d getPoseAgainstPodium() {
        if (!DriverStation.getAlliance().isEmpty()) {
            switch (DriverStation.getAlliance().get()) {
                case Blue:
                    return FieldConstants.synced.getObject().robotAgainstBluePodium;
                case Red:
                    return FieldConstants.synced.getObject().robotAgainstRedPodium;
            }
        }
        return FieldConstants.synced.getObject().robotAgainstRedPodium;
    }

    public static Pose2d getPoseAgainstAmpZone() {
        if (!DriverStation.getAlliance().isEmpty()) {
            switch (DriverStation.getAlliance().get()) {
                case Blue:
                    return FieldConstants.synced.getObject().robotAgainstRedAmpZone;
                case Red:
                    return FieldConstants.synced.getObject().robotAgainstBlueAmpZone;
            }
        }
        return FieldConstants.synced.getObject().robotAgainstRedAmpZone;
    }

    public static Rotation2d getSourceHeading() {
        if (!DriverStation.getAlliance().isEmpty()) {
            switch (DriverStation.getAlliance().get()) {
                case Blue:
                    Logger.recordOutput(
                            "Field/source", FieldConstants.synced.getObject().blueSourceHeading);
                    return FieldConstants.synced.getObject().blueSourceHeading;
                case Red:
                    Logger.recordOutput(
                            "Field/source", FieldConstants.synced.getObject().redSourceHeading);
                    return FieldConstants.synced.getObject().redSourceHeading;
            }
        }
        return FieldConstants.synced.getObject().redSourceHeading;
    }

    /** Returns whether the speaker is significantly to the robot's left */
    public static boolean isLeftOfSpeaker(double robotY, double tolerance) {
        if (!DriverStation.getAlliance().isEmpty()) {
            switch (DriverStation.getAlliance().get()) {
                case Blue:
                    return robotY
                            > FieldConstants.synced.getObject().fieldToBlueSpeaker.getY()
                                    + tolerance;
                case Red:
                    return robotY
                            < FieldConstants.synced.getObject().fieldToRedSpeaker.getY()
                                    - tolerance;
            }
        }
        return robotY < FieldConstants.synced.getObject().fieldToRedSpeaker.getY() - tolerance;
    }

    /** Returns whether the speaker is significantly to the robot's right */
    public static boolean isRightOfSpeaker(double robotY, double tolerance) {
        if (!DriverStation.getAlliance().isEmpty()) {
            switch (DriverStation.getAlliance().get()) {
                case Blue:
                    return robotY
                            < FieldConstants.synced.getObject().fieldToBlueSpeaker.getY()
                                    - tolerance;
                case Red:
                    return robotY
                            > FieldConstants.synced.getObject().fieldToRedSpeaker.getY()
                                    + tolerance;
            }
        }
        return robotY > FieldConstants.synced.getObject().fieldToRedSpeaker.getY() + tolerance;
    }
}
