package frc.robot.subsystems.localization;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.VisionConstants.CameraParams;
import frc.robot.subsystems.localization.VisionLocalizer.CameraMeasurement;
import org.littletonrobotics.junction.Logger;

public class Camera {

    public enum CameraTrustZone {
        LEFT,
        RIGHT,
        MIDDLE,
    }

    public final String name;
    public final CameraTrustZone zone;

    private final CameraIO io;
    private final CameraIOInputsAutoLogged inputs;

    private double distanceToXweighting = VisionConstants.distanceToXweighting;
    private double distanceToYweighting = VisionConstants.distanceToYweighting;
    private double distanceToHeadingweighting = VisionConstants.distanceToHeadingweighting;

    private double headingToXweighting = VisionConstants.headingToXweighting;
    private double headingToYweighting = VisionConstants.headingToYweighting;
    private double headingToHeadingweighting = VisionConstants.headingToHeadingweighting;

    private double ambiguityweighting = VisionConstants.ambiguityweighting;

    private double ntagsweighting = VisionConstants.ntagsweighting;

    public Camera(CameraParams params, CameraIO io) {
        name = params.name();
        zone = params.zone();

        this.io = io;
        inputs = new CameraIOInputsAutoLogged();
    }

    public void update() {
        io.updateInputs(inputs);
        Logger.processInputs("Vision/" + name, inputs);
    }

    public boolean hasNewMeasurement() {
        return inputs.wasAccepted;
    }

    public boolean isConnected() {
        return inputs.connected;
    }

    public CameraMeasurement getLatestMeasurement() {
        return new CameraMeasurement(
                inputs.latestFieldToRobot,
                inputs.latestTimestampSeconds,
                getLatestStandardDeviation());
    }

    /* original variance class: keeping for comparison
    public Matrix<N3, N1> getLatestVariance() {
        // If the robot is not in teleop, trust cameras based on their location relative to the tags
        if (!DriverStation.isTeleop()) {
            switch (zone) {
                case LEFT:
                    if (AllianceUtil.isRightOfSpeaker(inputs.latestFieldToRobot.getY(), 2)) {
                        return VisionConstants.lowCameraUncertainty;
                    }
                    break;
                case RIGHT:
                    if (AllianceUtil.isLeftOfSpeaker(inputs.latestFieldToRobot.getY(), 2)) {
                        return VisionConstants.lowCameraUncertainty;
                    }
                    break;
                case MIDDLE:
                    break;
            }
        }

        // If the robot is very close, trust highly
        if (inputs.averageTagDistanceM < VisionConstants.skewCutoffDistance) {
            return DriverStation.isTeleop()
                    ? VisionConstants.teleopCameraUncertainty
                    : VisionConstants.lowCameraUncertainty;
            // If the robot is somewhat close, check if the cameras are at extreme angles, and trust
            // accordingly
        } else if (inputs.averageTagDistanceM < VisionConstants.lowUncertaintyCutoffDistance) {
            return Math.abs(inputs.averageTagYaw.getDegrees()) < VisionConstants.skewCutoffRotation
                    ? VisionConstants.lowCameraUncertainty
                    : VisionConstants.highCameraUncertainty;
            // If the robot is past the final distance cutoff, distrust
        } else {
            return VisionConstants.highCameraUncertainty;
        }
    }*/

    public Matrix<N3, N1> getLatestStandardDeviation() {

        double xV = inputs.standardDeviationOfTags[0];
        double yV = inputs.standardDeviationOfTags[1];
        double headingV = inputs.standardDeviationOfTags[2];

        // distance error
        double distance = inputs.averageTagDistanceM;
        xV += Math.pow((distance / distanceToXweighting), 2);
        yV += Math.pow((distance / distanceToYweighting), 2);
        headingV += Math.pow((distance / distanceToHeadingweighting), 2);

        // distance error
        double heading = inputs.averageTagYaw.getDegrees();
        xV += Math.pow((heading / headingToXweighting), 2);
        yV += Math.pow((heading / headingToYweighting), 2);
        headingV += Math.pow((heading / headingToHeadingweighting), 2);

        // ambiguity error
        double ambiguity = inputs.ambiguity;
        xV += Math.pow(ambiguity * ambiguityweighting, 2);
        yV += Math.pow(ambiguity * ambiguityweighting, 2);
        heading += Math.pow(ambiguity * ambiguityweighting, 2);

        // ntags error
        double nTags = inputs.nTags;
        xV += Math.pow(nTags / ntagsweighting, 2);
        yV += Math.pow(nTags / ntagsweighting, 2);
        heading += Math.pow(nTags / ntagsweighting, 2);

        return VecBuilder.fill(Math.sqrt(xV), Math.sqrt(yV), Math.sqrt(headingV));
    }

    public void updateWeightings(double[] weightings) {

        distanceToXweighting = weightings[0];
        distanceToYweighting = weightings[1];
        distanceToHeadingweighting = weightings[2];

        headingToXweighting = weightings[3];
        headingToYweighting = weightings[4];
        headingToHeadingweighting = weightings[5];

        ambiguityweighting = weightings[6];
        ntagsweighting = weightings[7];
    }
}
