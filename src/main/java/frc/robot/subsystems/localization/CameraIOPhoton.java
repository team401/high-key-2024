package frc.robot.subsystems.localization;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.VisionConstants.CameraParams;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class CameraIOPhoton implements CameraIO {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;

    private double latestTimestampSeconds = 0.0;

    public CameraIOPhoton(String name, Transform3d robotToCamera) {
        this(new PhotonCamera(name), robotToCamera);
    }

    public CameraIOPhoton(PhotonCamera camera, Transform3d robotToCamera) {
        this.camera = camera;

        poseEstimator =
                new PhotonPoseEstimator(
                        VisionConstants.fieldLayout,
                        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                        robotToCamera);
    }

    public static CameraIOPhoton fromRealCameraParams(CameraParams params) {
        return new CameraIOPhoton(params.name(), params.robotToCamera());
    }

    public static CameraIOPhoton fromSimCameraParams(
            CameraParams params, VisionSystemSim sim, boolean stream) {
        PhotonCamera camera = new PhotonCamera(params.name());

        SimCameraProperties props = new SimCameraProperties();
        props.setCalibration(params.xResolution(), params.yResolution(), params.fov());
        props.setFPS(params.fps());
        props.setCalibError(0.25, 0.08);

        PhotonCameraSim cameraSim = new PhotonCameraSim(camera, props);
        sim.addCamera(cameraSim, params.robotToCamera());

        cameraSim.enableRawStream(stream);
        cameraSim.enableProcessedStream(stream);

        return new CameraIOPhoton(camera, params.robotToCamera());
    }

    @Override
    public void updateInputs(CameraIOInputs inputs) {
        inputs.connected = camera.isConnected();

        PhotonPipelineResult result = camera.getLatestResult();
        if (result.getTimestampSeconds() == latestTimestampSeconds) {
            inputs.isNewMeasurement = false;
            inputs.wasAccepted = false;
            return;
        }
        inputs.isNewMeasurement = true;
        latestTimestampSeconds = result.getTimestampSeconds();
        Optional<EstimatedRobotPose> photonPose = poseEstimator.update(filterTargets(result));

        photonPose.filter(CameraIOPhoton::filterPhotonPose);

        photonPose.ifPresentOrElse(
                (pose) -> {
                    inputs.latestFieldToRobot = pose.estimatedPose.toPose2d();
                    inputs.nTags = pose.targetsUsed.size();

                    inputs.latestTimestampSeconds = this.latestTimestampSeconds;
                    inputs.latencySeconds = result.getLatencyMillis() / 1000.0;
                    inputs.averageTagDistanceM = calculateAverageTagDistance(pose);
                    inputs.averageTagYaw = calculateAverageTagYaw(pose);

                    inputs.wasAccepted = true;
                },
                () -> {
                    inputs.wasAccepted = false;
                });
    }

    private static PhotonPipelineResult filterTargets(PhotonPipelineResult unfiltered) {
        List<PhotonTrackedTarget> targets = unfiltered.getTargets();
        List<PhotonTrackedTarget> filteredTargets = new ArrayList<PhotonTrackedTarget>();

        for (PhotonTrackedTarget target : targets) {
            if (target.getPoseAmbiguity() < VisionConstants.maximumAmbiguity
                    && target.getPitch() < VisionConstants.maximumPitch) {
                filteredTargets.add(target);
            }
        }
        return new PhotonPipelineResult(unfiltered.getLatencyMillis(), filteredTargets);
    }

    private static boolean filterPhotonPose(EstimatedRobotPose photonPose) {
        if (photonPose.targetsUsed.size() < 2) {
            return false;
        }

        Pose3d pose = photonPose.estimatedPose;
        // check that the pose isn't insane
        if (pose.getZ() > 1 || pose.getZ() < -0.1) {
            return false;
        }

        // TODO: Figure out if a max distance cap is good or necessary
        if (calculateAverageTagDistance(photonPose) > VisionConstants.maxTagDistance) {
            return false;
        }

        return true;
    }

    private static double calculateAverageTagDistance(EstimatedRobotPose pose) {
        double distance = 0.0;
        for (PhotonTrackedTarget target : pose.targetsUsed) {
            distance +=
                    target.getBestCameraToTarget()
                            .getTranslation()
                            .getDistance(new Translation3d());
        }
        distance /= pose.targetsUsed.size();

        return distance;
    }

    private static Rotation2d calculateAverageTagYaw(EstimatedRobotPose pose) {
        double yawRad = 0.0;
        for (PhotonTrackedTarget target : pose.targetsUsed) {
            yawRad += target.getBestCameraToTarget().getRotation().getZ();
        }
        yawRad /= pose.targetsUsed.size();
        yawRad -= Math.PI * Math.signum(yawRad);

        return Rotation2d.fromRadians(yawRad);
    }
}
