package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.IOException;

public class VisionConstants {
  public static AprilTagFieldLayout aprilTagLayout;

  static {
    try {
      aprilTagLayout =
          new AprilTagFieldLayout(
              new File(Filesystem.getDeployDirectory(), "BunnybotsField.json").toPath());
    } catch (IOException e) {
      throw new RuntimeException("Failed to load AprilTag layout", e);
    }
  }

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "camera_0";
  public static String camera1Name = "camera_1";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCamera0 =
      new Transform3d(
          Units.inchesToMeters(11.12),
          -Units.inchesToMeters(9.77),
          Units.inchesToMeters(5.98),
          new Rotation3d(0.0, -Units.degreesToRadians(20), 0));
  public static Transform3d robotToCamera1 =
      new Transform3d(
          Units.inchesToMeters(1.7),
          Units.inchesToMeters(.24),
          Units.inchesToMeters(36.7),
          new Rotation3d(0.0, -0.0, Math.PI));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        2.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
