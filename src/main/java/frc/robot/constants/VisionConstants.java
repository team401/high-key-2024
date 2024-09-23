package frc.robot.constants;

import coppercore.parameter_tools.JSONExclude;
import coppercore.parameter_tools.JSONSync;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
// import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.localization.Camera.CameraTrustZone;
import java.io.IOException;
import java.util.Collections;
import java.util.List;

public final class VisionConstants {

    @JSONExclude
    public static final JSONSync<VisionConstants> synced =
            new JSONSync<VisionConstants>(
                    new VisionConstants(),
                    "filePath",
                    new JSONSync.JSONSyncConfigBuilder().build());

    public final String tagLayoutName = "Pairs-Only";
    public final AprilTagFieldLayout fieldLayout = initLayout(tagLayoutName);

    public final double lowUncertaintyCutoffDistance = 6.5;

    public final double skewCutoffDistance = 5.8;
    public final double skewCutoffRotation = Units.degreesToRadians(50);

    // Maximum average tag distance before a measurement is fully ignored
    public final double maxTagDistance = 8.0;

    public final Matrix<N3, N1> teleopCameraUncertainty = VecBuilder.fill(0.35, 0.35, 3.5);

    public final Matrix<N3, N1> lowCameraUncertainty = VecBuilder.fill(0.6, 1.0, 4);

    public final Matrix<N3, N1> highCameraUncertainty = VecBuilder.fill(12.0, 16.0, 40);

    public final Matrix<N3, N1> driveUncertainty = VecBuilder.fill(0.1, 0.1, 0.1);

    public final List<CameraParams> cameras =
            List.of(
                    new CameraParams(
                            "Front-Left",
                            1280,
                            960,
                            50,
                            Rotation2d.fromDegrees(70),
                            new Transform3d(
                                    new Translation3d(0.306, 0.259, 0.211),
                                    new Rotation3d(0, -0.349, 0.785)),
                            CameraTrustZone.LEFT));

    //     new CameraParams(
    //             "Front-Right",
    //             1280,
    //             960,
    //             50,
    //             Rotation2d.fromDegrees(70),
    //             new Transform3d(
    //                     new Translation3d(0.312, -0.304, 0.217),
    //                     new Rotation3d(0.0, -0.349, -0.785)),
    //             CameraTrustZone.RIGHT),
    //     new CameraParams(
    //             "Front-Center",
    //             1280,
    //             960,
    //             50,
    //             Rotation2d.fromDegrees(70),
    //             new Transform3d(
    //                     new Translation3d(0.312, -0.237, 0.233),
    //                     new Rotation3d(0.0, -0.349, 0.524)),
    //             CameraTrustZone.MIDDLE));

    public record CameraParams(
            String name,
            int xResolution,
            int yResolution,
            int fps,
            Rotation2d fov,
            Transform3d robotToCamera,
            CameraTrustZone zone) {}

    private AprilTagFieldLayout initLayout(String name) {
        AprilTagFieldLayout layout;
        // AprilTagFieldLayout's constructor throws an IOException, so we must catch it
        // in order to initialize our layout as a static constant
        try {
            layout =
                    new AprilTagFieldLayout(
                            Filesystem.getDeployDirectory().getAbsolutePath()
                                    + "/taglayout/"
                                    + name
                                    + ".json");
        } catch (IOException ioe) {
            DriverStation.reportWarning(
                    "Failed to load AprilTag Layout: " + ioe.getLocalizedMessage(), false);
            layout = new AprilTagFieldLayout(Collections.emptyList(), 0.0, 0.0);
        }
        return layout;
    }
}
