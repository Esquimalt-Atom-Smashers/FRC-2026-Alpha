package frc.robot.subsystems.shooter.turret;

import org.littletonrobotics.junction.AutoLog;

/** IO interface for the turret (one motor + encoder). */
public interface TurretIO {

  @AutoLog
  class TurretIOInputs {
    public boolean motorConnected = false;
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
  }

  /** Update inputs from the hardware. */
  default void updateInputs(TurretIOInputs inputs) {}

  /** Set motor output voltage (e.g. from PID). */
  default void setVoltage(double volts) {}
}
