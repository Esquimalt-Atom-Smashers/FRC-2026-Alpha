// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.nio.file.Path;

public class VisionConstants {
  // AprilTag layout  TODO: Uncomment this if/when WPILib adds 2026-rebuilt-welded.json
  // public static AprilTagFieldLayout aprilTagLayout =
  // AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Load the AprilTag field layout
  // Note: WPILib 2026 doesn't include 2026 field layouts in AprilTagFields enum yet,
  // so we load the custom 2026 field layout JSON from deploy directory
  // File location: src/main/deploy/2026-rebuilt-welded.json (deployed to /home/lvuser/deploy/ on
  // roboRIO)
  public static AprilTagFieldLayout aprilTagLayout;

  static {
    // On real robot, files in src/main/deploy are deployed to /home/lvuser/deploy/
    // Use Filesystem.getDeployDirectory() to get the correct path
    try {
      Path deployPath =
          Filesystem.getDeployDirectory().toPath().resolve("2026-rebuilt-welded.json");
      aprilTagLayout = new AprilTagFieldLayout(deployPath.toString());
    } catch (Exception e) {
      DriverStation.reportError(
          "Failed to load AprilTag field layout: " + e.getMessage(), e.getStackTrace());
      aprilTagLayout = null;
    }
  }

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "Arducam_OV9821";
  public static String camera1Name = "webcam_0";

  // Robot to camera transforms
  public static Transform3d robotToCamera0 =
      new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0));
  public static Transform3d robotToCamera1 =
      new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));

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
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
