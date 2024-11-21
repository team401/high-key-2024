package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.ConstantsLoader.FieldConstants;
import org.littletonrobotics.junction.Logger;

public class AllianceUtil {

    public static Translation2d getFieldToSpeaker() {
        if (!DriverStation.getAlliance().isEmpty()) {
            switch (DriverStation.getAlliance().get()) {
                case Blue:
                    Logger.recordOutput("Field/speaker", ConstantsLoader.FieldConstants.fieldToBlueSpeaker);
                    return ConstantsLoader.FieldConstants.fieldToBlueSpeaker;
                case Red:
                    Logger.recordOutput("Field/speaker", ConstantsLoader.FieldConstants.fieldToRedSpeaker);
                    return ConstantsLoader.FieldConstants.fieldToRedSpeaker;
            }
        }
        return ConstantsLoader.FieldConstants.fieldToRedSpeaker;
    }

    public static Rotation2d getAmpHeading() {
        Logger.recordOutput("Field/amp", ConstantsLoader.FieldConstants.ampHeading);
        return ConstantsLoader.FieldConstants.ampHeading;
    }

    public static Pose2d getPoseAgainstSpeaker() {
        if (!DriverStation.getAlliance().isEmpty()) {
            switch (DriverStation.getAlliance().get()) {
                case Blue:
                    return ConstantsLoader.FieldConstants.robotAgainstBlueSpeaker;
                case Red:
                    return ConstantsLoader.FieldConstants.robotAgainstRedSpeaker;
            }
        }
        return ConstantsLoader.FieldConstants.robotAgainstRedSpeaker;
    }

    public static Pose2d getPoseAgainstSpeakerLeft() {
        if (!DriverStation.getAlliance().isEmpty()) {
            switch (DriverStation.getAlliance().get()) {
                case Blue:
                    return ConstantsLoader.FieldConstants.robotAgainstBlueSpeakerLeft;
                case Red:
                    return ConstantsLoader.FieldConstants.robotAgainstRedSpeakerLeft;
            }
        }
        return ConstantsLoader.FieldConstants.robotAgainstRedSpeakerLeft;
    }

    public static Pose2d getPoseAgainstSpeakerRight() {
        if (!DriverStation.getAlliance().isEmpty()) {
            switch (DriverStation.getAlliance().get()) {
                case Blue:
                    return ConstantsLoader.FieldConstants.robotAgainstBlueSpeakerRight;
                case Red:
                    return ConstantsLoader.FieldConstants.robotAgainstRedSpeakerRight;
            }
        }
        return ConstantsLoader.FieldConstants.robotAgainstRedSpeakerRight;
    }

    public static Pose2d getPoseAgainstPodium() {
        if (!DriverStation.getAlliance().isEmpty()) {
            switch (DriverStation.getAlliance().get()) {
                case Blue:
                    return ConstantsLoader.FieldConstants.robotAgainstBluePodium;
                case Red:
                    return ConstantsLoader.FieldConstants.robotAgainstRedPodium;
            }
        }
        return ConstantsLoader.FieldConstants.robotAgainstRedPodium;
    }

    public static Pose2d getPoseAgainstAmpZone() {
        if (!DriverStation.getAlliance().isEmpty()) {
            switch (DriverStation.getAlliance().get()) {
                case Blue:
                    return ConstantsLoader.FieldConstants.robotAgainstRedAmpZone;
                case Red:
                    return ConstantsLoader.FieldConstants.robotAgainstBlueAmpZone;
            }
        }
        return ConstantsLoader.FieldConstants.robotAgainstRedAmpZone;
    }

    public static Rotation2d getSourceHeading() {
        if (!DriverStation.getAlliance().isEmpty()) {
            switch (DriverStation.getAlliance().get()) {
                case Blue:
                    Logger.recordOutput("Field/source", ConstantsLoader.FieldConstants.blueSourceHeading);
                    return ConstantsLoader.FieldConstants.blueSourceHeading;
                case Red:
                    Logger.recordOutput("Field/source", ConstantsLoader.FieldConstants.redSourceHeading);
                    return ConstantsLoader.FieldConstants.redSourceHeading;
            }
        }
        return ConstantsLoader.FieldConstants.redSourceHeading;
    }

    /** Returns whether the speaker is significantly to the robot's left */
    public static boolean isLeftOfSpeaker(double robotY, double tolerance) {
        if (!DriverStation.getAlliance().isEmpty()) {
            switch (DriverStation.getAlliance().get()) {
                case Blue:
                    return robotY > ConstantsLoader.FieldConstants.fieldToBlueSpeaker.getY() + tolerance;
                case Red:
                    return robotY < ConstantsLoader.FieldConstants.fieldToRedSpeaker.getY() - tolerance;
            }
        }
        return robotY < ConstantsLoader.FieldConstants.fieldToRedSpeaker.getY() - tolerance;
    }

    /** Returns whether the speaker is significantly to the robot's right */
    public static boolean isRightOfSpeaker(double robotY, double tolerance) {
        if (!DriverStation.getAlliance().isEmpty()) {
            switch (DriverStation.getAlliance().get()) {
                case Blue:
                    return robotY < ConstantsLoader.FieldConstants.fieldToBlueSpeaker.getY() - tolerance;
                case Red:
                    return robotY > ConstantsLoader.FieldConstants.fieldToRedSpeaker.getY() + tolerance;
            }
        }
        return robotY > ConstantsLoader.FieldConstants.fieldToRedSpeaker.getY() + tolerance;
    }
}
