package frc.robot.subsystems.shooter.transfer;

import org.littletonrobotics.junction.AutoLog;

/** IO interface for the transfer (motor + colour sensor), voltage controlled. */
public interface TransferIO {

  @AutoLog
  class TransferIOInputs {
    public boolean motorConnected = false;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public boolean colorSensorTripped = false;
  }

  /** Update inputs from the hardware. */
  default void updateInputs(TransferIOInputs inputs) {}

  /** Set the motor output voltage (V). */
  default void setVoltage(double volts) {}

  /** Stop the motor (coast). */
  default void stop() {}
}
