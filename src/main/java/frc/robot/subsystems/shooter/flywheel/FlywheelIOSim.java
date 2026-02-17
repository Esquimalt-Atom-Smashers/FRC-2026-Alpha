package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;

/** Flywheel IO for simulation; slew-rate-limited setpoint following. */
public class FlywheelIOSim implements FlywheelIO {

  /** Max acceleration: 4000 RPM in 2 s (rad/s²). */
  private static final double kMaxAccelRadPerSecSq = Units.rotationsPerMinuteToRadiansPerSecond(4000.0) / 2.0;

  private final SlewRateLimiter slewRateLimiter = new SlewRateLimiter(kMaxAccelRadPerSecSq);

  private double targetVelocityRadsPerSec = 0.0;
  private boolean isStopped = false;

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    if (isStopped) {
      targetVelocityRadsPerSec = 0.0;
    }
    double currentVelocityRadsPerSec = slewRateLimiter.calculate(targetVelocityRadsPerSec);

    inputs.motorConnected = true;
    inputs.velocityRadsPerSec = currentVelocityRadsPerSec;
    inputs.appliedVolts = 0.0;
    inputs.supplyCurrentAmps = 0.0;
  } // End updateInputs

  @Override
  public void setTargetVelocity(double targetVelocityRadsPerSec) {
    this.targetVelocityRadsPerSec = targetVelocityRadsPerSec;
    isStopped = false;
  } // End setTargetVelocity

  @Override
  public void stop() {
    isStopped = true;
  } // End stop
}
