package frc.robot.subsystems.shooter.transfer;

import edu.wpi.first.math.util.Units;

/** Constants for the transfer (agitator-to-shooter) subsystem. */
public final class TransferConstants { // TODO: Add correct values

  private TransferConstants() {}

  /** CAN ID of the transfer motor. */
  public static final int kMotorId = 0;

  /** Output rotations per motor rotation (1.0 = 1:1). */
  public static final double kGearRatio = 1.0;

  /** Max voltage magnitude applied to the motor. */
  public static final double kMaxVoltage = 12.0;

  /** Set true if positive velocity spins the transfer the opposite direction. */
  public static final boolean kMotorInverted = false;

  /** Velocity (rad/s) when staging (slow pre-load). */
  public static final double kStagingVelocityRadsPerSec =
      Units.rotationsPerMinuteToRadiansPerSecond(200.0);

  /** Velocity (rad/s) when shooting. */
  public static final double kShootingVelocityRadsPerSec =
      Units.rotationsPerMinuteToRadiansPerSecond(3000.0);

  /** Velocity PIDF gains (onboard and sim). */
  public static final double kP = 0.0001;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kV = 0.0002;
  public static final double kS = 0.0;

  /** Proximity at or above this value = ball present (REV Color Sensor V3: 0–2047). */
  public static final int kColorSensorProximityThreshold = 150;
}

