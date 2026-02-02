package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.ShooterLookup;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.util.AllianceUtil;
import org.littletonrobotics.junction.Logger;

/** Shooter-related command helpers (hood, flywheel, distance-to-hub lookup). */
public final class ShooterCommands {

  private ShooterCommands() {}

  /**
   * Sets hood angle and flywheel target from distance-to-hub lookup, and logs distance, angle,
   * and RPM.
   */
  public static void setShooterTarget(Drive drive, Hood hood, Flywheel flywheel) {
    double distanceMeters = getDistanceToHubMeters(drive);
    ShooterLookup.Result lookupResult = ShooterLookup.get(distanceMeters);

    Logger.recordOutput("Shooter/DistanceToHubMeters", distanceMeters);
    Logger.recordOutput(
        "Shooter/LookupAngleDegrees",
        Units.radiansToDegrees(lookupResult.angleRad()));
    Logger.recordOutput(
        "Shooter/LookupVelocityRpm",
        Units.radiansPerSecondToRotationsPerMinute(lookupResult.velocityRadsPerSec()));

    hood.setTargetAngleRad(lookupResult.angleRad());
    flywheel.setTargetVelocityRadsPerSec(lookupResult.velocityRadsPerSec());
  } // End setShooterTarget

  /**
   * Returns distance from robot to alliance hub center (meters). Used by shooter lookup (hood,
   * flywheel).
   */
  public static double getDistanceToHubMeters(Drive drive) {
    Pose2d robotPose = drive.getPose();
    Translation2d robotPosition = robotPose.getTranslation();
    Translation2d targetPosition =
        AllianceUtil.isRedAlliance() ? FieldConstants.RED_HUB_CENTER : FieldConstants.BLUE_HUB_CENTER;
    return targetPosition.minus(robotPosition).getNorm();
  } // End getDistanceToHubMeters
}
