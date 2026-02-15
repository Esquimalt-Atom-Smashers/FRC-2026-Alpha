package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/** Field layout constants including target positions for different alliances. */
public final class FieldConstants {

  /** Blue alliance hub center position in meters (2D, for pathfinding etc.). */
  public static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.6256194, 4.0346376);

  /** Red alliance hub center position in meters (2D, for pathfinding etc.). */
  public static final Translation2d RED_HUB_CENTER = new Translation2d(11.9154194, 4.0346376);

  /** Hub center height above floor. */
  private static final double HUB_CENTER_HEIGHT_M = 1.43256;

  /** Blue alliance hub center in 3D (x, y, z) meters for shooter ballistics. */
  public static final Translation3d BLUE_HUB_CENTER_3D =
      new Translation3d(BLUE_HUB_CENTER.getX(), BLUE_HUB_CENTER.getY(), HUB_CENTER_HEIGHT_M);

  /** Red alliance hub center in 3D (x, y, z) meters for shooter ballistics. */
  public static final Translation3d RED_HUB_CENTER_3D =
      new Translation3d(RED_HUB_CENTER.getX(), RED_HUB_CENTER.getY(), HUB_CENTER_HEIGHT_M);

  /** Funnel radius. */
  public static final double FUNNEL_RADIUS_M = 0.6096;

  /** Funnel height. */
  public static final double FUNNEL_HEIGHT_M = 0.39624;

  /** Funnel top height above floor. */
  public static final double FUNNEL_TOP_HEIGHT_M = HUB_CENTER_HEIGHT_M + FUNNEL_HEIGHT_M;

  /** Blue alliance funnel top center in 3D (x, y, z) for shooter ballistics. */
  public static final Translation3d BLUE_FUNNEL_TOP_CENTER_3D =
      new Translation3d(BLUE_HUB_CENTER.getX(), BLUE_HUB_CENTER.getY(), FUNNEL_TOP_HEIGHT_M);

  /** Red alliance funnel top center in 3D (x, y, z) for shooter ballistics. */
  public static final Translation3d RED_FUNNEL_TOP_CENTER_3D =
      new Translation3d(RED_HUB_CENTER.getX(), RED_HUB_CENTER.getY(), FUNNEL_TOP_HEIGHT_M);

  /** Alliance zone depth. */
  public static final double ALLIANCE_ZONE_M = 3.977927;

  /** Passing spot left (90 in from driver wall, 85 in left of field center). */
  public static final Translation3d PASSING_SPOT_LEFT = new Translation3d(2.286, 6.180328, 0);

  /** Passing spot center (90 in from driver wall, field center y). */
  public static final Translation3d PASSING_SPOT_CENTER = new Translation3d(2.286, 4.021328, 0);

  /** Passing spot right (90 in from driver wall, 85 in right of field center). */
  public static final Translation3d PASSING_SPOT_RIGHT = new Translation3d(2.286, 1.862328, 0);

  private FieldConstants() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
