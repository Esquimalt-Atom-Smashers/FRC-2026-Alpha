package frc.robot.subsystems.shooter.turrent;

import static frc.robot.subsystems.shooter.turrent.TurretConstants.kGearRatio;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Turret IO implementation for simulation. */
public class TurretIOSim implements TurretIO {

  private static final double kLoopPeriodSecs = 0.02;

  private final DCMotor gearbox = DCMotor.getKrakenX44Foc(1);
  private final DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(gearbox, 0.001, 100.0 / kGearRatio), gearbox);

  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    sim.update(kLoopPeriodSecs);

    inputs.motorConnected = true;
    inputs.positionRads = sim.getAngularPositionRad() * kGearRatio;
    inputs.velocityRadsPerSec = sim.getAngularVelocityRadPerSec() * kGearRatio;
    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
  }
}
