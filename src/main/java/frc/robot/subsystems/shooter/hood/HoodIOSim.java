package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;

/** Hood IO for simulation; slew-rate-limited setpoint following. */
public class HoodIOSim implements HoodIO {

  /** Max setpoint slew rate (rad/s); full range ~30° in ~0.5 s. */
  private static final double kMaxAngularRateRadPerSec = Units.degreesToRadians(60.0);

  private final SlewRateLimiter slewRateLimiter = new SlewRateLimiter(kMaxAngularRateRadPerSec);

  private double targetPositionRad = 0.0;
  private double limitedPositionRad = HoodConstants.kMinAngleRad;
  private boolean isStopped = false;

  public HoodIOSim() {
    slewRateLimiter.reset(HoodConstants.kMinAngleRad);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    if (!isStopped) {
      double clampedTarget = MathUtil.clamp(targetPositionRad, HoodConstants.kMinAngleRad, HoodConstants.kMaxAngleRad);
      limitedPositionRad = slewRateLimiter.calculate(clampedTarget);
    }

    inputs.motorConnected = true;
    inputs.positionRads = limitedPositionRad;
    inputs.velocityRadsPerSec = 0.0;
    inputs.appliedVolts = 0.0;
    inputs.supplyCurrentAmps = 0.0;
  } // End updateInputs

  @Override
  public void setTargetPosition(double targetRads) {
    this.targetPositionRad = targetRads;
    isStopped = false;
  } // End setTargetPosition

  @Override
  public void stop() {
    isStopped = true;
  } // End stop
}
