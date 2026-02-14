package frc.robot.subsystems.shooter.transfer;

/** Constants for the transfer (agitator-to-shooter) subsystem. */
public final class TransferConstants { // TODO: Add correct values

  private TransferConstants() {}

  /** CAN ID of the transfer motor. */
  public static final int kMotorId = 7;

  /** Max voltage magnitude applied to the motor. */
  public static final double kMaxVoltage = 10.0;

  /** Set true if positive voltage spins the transfer the opposite direction. */
  public static final boolean kMotorInverted = false;

  /** Voltage when idle. */
  public static final double kIdleVoltage = 0.0;

  /** Voltage when staging (slow pre-load). */
  public static final double kStagingVoltage = 2.0;

  /** Voltage when shooting. */
  public static final double kShootingVoltage = 6.0;

  /** Proximity at or above this value = ball present (REV Color Sensor V3: 0–2047). */
  public static final int kColorSensorProximityThreshold = 150;
}
