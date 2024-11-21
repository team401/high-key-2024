package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
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

public  class VisionConstants {
    public static final   String tagLayoutName = "Pairs-Only";
    public static final   AprilTagFieldLayout fieldLayout = initLayout(tagLayoutName);

    public  static final  double lowUncertaintyCutoffDistance = 6.5;

    public static final   double skewCutoffDistance = 5.8;
    public static final   double skewCutoffRotation = Units.degreesToRadians(50);

    // Maximum average tag distance before a measurement is fully ignored
    public static final   double maxTagDistance = 8.0;

    public static final   Matrix<N3, N1> teleopCameraUncertainty = VecBuilder.fill(0.35, 0.35, 3.5);

    public static final   Matrix<N3, N1> lowCameraUncertainty = VecBuilder.fill(0.6, 1.0, 4);

    public static final   Matrix<N3, N1> highCameraUncertainty = VecBuilder.fill(12.0, 16.0, 40);

    public static final   Matrix<N3, N1> driveUncertainty = VecBuilder.fill(0.1, 0.1, 0.1);

    public static final   List<CameraParams> cameras = List.of();

    //     new CameraParams(
    //             "Front-Left", // Front Right
    //             1280,
    //             960,
    //             50,
    //             Rotation2d.fromDegrees(70),
    //             new Transform3d(
    //                     new Translation3d(0.0328422, -0.3103626, 0.430911),
    //                     new Rotation3d(0, -0.261799, 0.0)),
    //             CameraTrustZone.MIDDLE));

    //     new CameraParams(
    //             "Back-Right", // Back Right
    //             1280,
    //             960,
    //             50,
    //             Rotation2d.fromDegrees(70),
    //             new Transform3d(
    //                     new Translation3d(-0.3160014, -0.2327402, 0.3163316),
    //                     new Rotation3d(0.0, -0.408546671, Math.PI)),
    //             CameraTrustZone.MIDDLE),
    //     new CameraParams(
    //             "Front-Right", // Front Left
    //             1280,
    //             960,
    //             50,
    //             Rotation2d.fromDegrees(70),
    //             new Transform3d(
    //                     new Translation3d(0.0328422, 0.3103626, 0.430911),
    //                     new Rotation3d(Math.PI, -0.261799, 0.0)),
    //             CameraTrustZone.MIDDLE));

    //     new CameraParams(
    //             "Front-Center", // Back Left
    //             1280,
    //             960,
    //             50,
    //             Rotation2d.fromDegrees(70),
    //             new Transform3d(
    //                     new Translation3d(-0.3160014, 0.2327402, 0.3163316),
    //                     new Rotation3d(0.0, -0.408546671, Math.PI)),
    //             CameraTrustZone.MIDDLE));

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

    public static final  record CameraParams(
            String name,
            int xResolution,
            int yResolution,
            int fps,
            Rotation2d fov,
            Transform3d robotToCamera,
            CameraTrustZone zone) {}

    private static final  AprilTagFieldLayout initLayout(String name) {
        AprilTagFieldLayout layout;
        // AprilTagFieldLayout's constructor throws an IOException, so we must catch it
        // in order to initialize our layout as a  constant
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
