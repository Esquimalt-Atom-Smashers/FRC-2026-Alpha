package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Alliance-related helpers used across the robot (drive, shooter, etc.). */
public final class AllianceUtil {

  private AllianceUtil() {}

  /**
   * Returns true if the robot is on the red alliance, false otherwise (blue or unknown).
   *
   * @return true if on red alliance, false otherwise
   */
  public static boolean isRedAlliance() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;
  } // End isRedAlliance
}
