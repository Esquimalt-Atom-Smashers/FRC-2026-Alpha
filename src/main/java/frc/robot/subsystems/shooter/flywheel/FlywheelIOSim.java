package frc.robot.subsystems.shooter.flywheel;

import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kGearRatio;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kP;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kI;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kD;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kV;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kS;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kMaxVoltage;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Flywheel IO for simulation; software velocity PIDF. */
public class FlywheelIOSim implements FlywheelIO {

  private static final double kLoopPeriodSecs = 0.02;

  private final DCMotor motorModel = DCMotor.getKrakenX44Foc(1); // TODO: Add correct motor model
  private final DCMotorSim motorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(motorModel, 0.001, 1.0 / kGearRatio),
          motorModel);

  private double targetVelocityRadsPerSec = 0.0;
  private double integralRotSec = 0.0;
  private double previousRotPerSec = 0.0;
  private double appliedVolts = 0.0;
  private boolean isStopped = false;

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    if (isStopped) {
      integralRotSec = 0.0;
      appliedVolts = 0.0;
      motorSim.setInputVoltage(0.0);
      previousRotPerSec = motorSim.getAngularVelocityRadPerSec() / kGearRatio / (2.0 * Math.PI);
    } else {
      double currentVelocityRadsPerSec =
          motorSim.getAngularVelocityRadPerSec() / kGearRatio;
      double targetRotPerSec = targetVelocityRadsPerSec / (2.0 * Math.PI);
      double currentRotPerSec = currentVelocityRadsPerSec / (2.0 * Math.PI);
      double errorRotPerSec = targetRotPerSec - currentRotPerSec;

      integralRotSec += errorRotPerSec * kLoopPeriodSecs;
      @SuppressWarnings("unused")
      double maxIntegralSec = kI > 1e-9 ? kMaxVoltage / kI : 1e6;
      integralRotSec = MathUtil.clamp(integralRotSec, -maxIntegralSec, maxIntegralSec);

      double velocityErrorDerivativeRotPerSecSq =
          -(currentRotPerSec - previousRotPerSec) / kLoopPeriodSecs;
      previousRotPerSec = currentRotPerSec;

      double staticFf = Math.signum(targetRotPerSec) * kS;
      double velocityFf = kV * targetRotPerSec;
      double pidOutput =
          kP * errorRotPerSec + kI * integralRotSec + kD * velocityErrorDerivativeRotPerSecSq;
      appliedVolts = MathUtil.clamp(staticFf + velocityFf + pidOutput, -kMaxVoltage, kMaxVoltage);
      motorSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    }
    motorSim.update(kLoopPeriodSecs);

    inputs.motorConnected = true;
    inputs.velocityRadsPerSec = motorSim.getAngularVelocityRadPerSec() / kGearRatio;
    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps = motorSim.getCurrentDrawAmps();
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
