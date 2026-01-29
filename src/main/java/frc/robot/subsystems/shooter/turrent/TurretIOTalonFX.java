package frc.robot.subsystems.shooter.turrent;

import static frc.robot.subsystems.shooter.turrent.TurretConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

/** Turret IO implementation using a single Talon FX (integrated sensor). */
public class TurretIOTalonFX implements TurretIO {

  private final TalonFX motor;
  private final VoltageOut voltageRequest = new VoltageOut(0);

  public TurretIOTalonFX() {
    motor = new TalonFX(kMotorId, TunerConstants.kCANBus);

    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(
            motor.getPosition(),
            motor.getVelocity(),
            motor.getMotorVoltage(),
            motor.getSupplyCurrent());
    inputs.motorConnected = status.equals(StatusCode.OK);
    inputs.positionRads =
        Units.rotationsToRadians(motor.getPosition().getValueAsDouble()) * kGearRatio;
    inputs.velocityRadsPerSec =
        Units.rotationsToRadians(motor.getVelocity().getValueAsDouble()) * kGearRatio;
    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.supplyCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }
}
