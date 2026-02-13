package frc.robot.subsystems.agitator;

import edu.wpi.first.math.util.Units;

/** Constants for the agitator (intake-to-shooter transfer) subsystem. */
public final class AgitatorConstants { // TODO: Add correct values

  private AgitatorConstants() {}

  /** CAN ID of the agitator motor (NEO 550 on SPARK MAX). */
  public static final int kMotorId = 0; // TODO: Set correct CAN ID

  /** Output rotations per motor rotation (1.0 = 1:1). */
  public static final double kGearRatio = 1.0; // TODO: Set if not 1:1

  /** Max voltage magnitude applied to the motor. */
  public static final double kMaxVoltage = 12.0;

  /** Set true if positive velocity spins the agitator the opposite direction. */
  public static final boolean kMotorInverted = false;

  /** Velocity (rad/s) when staging (slow pre-load). */
  public static final double kStagingVelocityRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(200.0);

  /** Velocity (rad/s) when shooting. */
  public static final double kShootingVelocityRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(3000.0);

  /** Velocity PIDF gains (onboard and sim). */
  public static final double kP = 0.0001;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kV = 0.0002;
  public static final double kS = 0.0;
}
