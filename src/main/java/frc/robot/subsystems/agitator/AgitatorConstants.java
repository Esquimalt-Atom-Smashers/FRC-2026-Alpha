package frc.robot.subsystems.agitator;

/** Constants for the agitator (intake-to-transfer) subsystem. */
public final class AgitatorConstants { // TODO: Add correct values

  private AgitatorConstants() {}

  /** CAN ID of the agitator motor (NEO 550 on SPARK MAX). */
  public static final int kMotorId = 20;

  /** Max voltage magnitude applied to the motor. */
  public static final double kMaxVoltage = 10.0;

  /** Set true if positive voltage spins the agitator the opposite direction. */
  public static final boolean kMotorInverted = false;

  /** Voltage when idle. */
  public static final double kIdleVoltage = 0.0;

  /** Voltage when staging (slow pre-load). */
  public static final double kStagingVoltage = 2.0;

  /** Voltage when shooting. */
  public static final double kShootingVoltage = 6.0;
}
