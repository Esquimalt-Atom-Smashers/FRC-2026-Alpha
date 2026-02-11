package frc.robot.subsystems.shooter.transfer;

import org.littletonrobotics.junction.AutoLog;

/** IO interface for the transfer (motor + colour sensor). */
public interface TransferIO {

  @AutoLog
  class TransferIOInputs {
    public boolean motorConnected = false;
    public double velocityRadsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public boolean colorSensorTripped = false;
  }

  /** Update inputs from the hardware. */
  default void updateInputs(TransferIOInputs inputs) {}

  /** Set the target velocity (rad/s, output shaft). */
  default void setTargetVelocity(double targetVelocityRadsPerSec) {}

  /** Stop the motor (coast). */
  default void stop() {}
}
