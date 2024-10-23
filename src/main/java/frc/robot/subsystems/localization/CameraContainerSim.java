package frc.robot.subsystems.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.PhoenixDriveConstants;
// import frc.robot.constants.DriveConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.VisionConstants.CameraParams;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.VisionSystemSim;

public class CameraContainerSim implements CameraContainer {

    private VisionSystemSim visionSim = new VisionSystemSim("main");

    private List<Camera> cameras = new ArrayList<>();

    private Supplier<SwerveModuleState[]> getModuleStates;
    private Pose2d latestOdometryPose;
    private SwerveModulePosition[] lastModulePositions =
            new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            };
    private Timer dtTimer = new Timer();

    public CameraContainerSim(
            List<CameraParams> params, Supplier<SwerveModuleState[]> getModuleStates) {
        this.getModuleStates = getModuleStates;

        visionSim.addAprilTags(VisionConstants.fieldLayout);

        // TODO: Get actual constant from DriveConstants after constants get fixed
        var pose = new Pose2d();
        latestOdometryPose =
                new Pose2d(
                        pose.getX(),
                        pose.getY(),
                        Rotation2d.fromRadians(pose.getRotation().getRadians()));

        for (CameraParams param : params) {
            cameras.add(
                    new Camera(param, CameraIOPhoton.fromSimCameraParams(param, visionSim, true)));
        }
    }

    public List<Camera> getCameras() {
        return cameras;
    }

    public void update() {
        updateOdometry();
        visionSim.update(latestOdometryPose);
        SmartDashboard.putData("PhotonSimField", visionSim.getDebugField());

        for (Camera camera : cameras) {
            camera.update();
        }
    }

    private void updateOdometry() {
        SwerveModulePosition[] deltas = new SwerveModulePosition[4];
        SwerveModuleState[] states = getModuleStates.get();

        double dt = dtTimer.get();
        dtTimer.reset();
        dtTimer.start();

        for (int i = 0; i < states.length; i++) {
            deltas[i] =
                    new SwerveModulePosition(
                            states[i].speedMetersPerSecond * dt
                                    - lastModulePositions[i].distanceMeters,
                            Rotation2d.fromRadians(
                                    states[i]
                                            .angle
                                            .minus(lastModulePositions[i].angle)
                                            .getRadians()));
        }

        Twist2d twist = PhoenixDriveConstants.kinematics.toTwist2d(deltas);
        latestOdometryPose = latestOdometryPose.exp(twist);

        Logger.recordOutput("Vision/GroundTruth", latestOdometryPose);
    }

    public void updateWeightings(double[] weightings) {};
}
