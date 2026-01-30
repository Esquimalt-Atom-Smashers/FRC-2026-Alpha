package frc.robot.subsystems.shooter.flywheel;

import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

/** Flywheel IO using a Talon FX with onboard velocity control. */
public class FlywheelIOTalonFX implements FlywheelIO {

  private final TalonFX motor;
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

  public FlywheelIOTalonFX() {
    motor = new TalonFX(kMotorId, TunerConstants.kCANBus);

    var talonFxConfig = new TalonFXConfiguration();
    talonFxConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    talonFxConfig.MotorOutput.Inverted =
        kMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    talonFxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFxConfig.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimitAmps;

    talonFxConfig.Slot0.kP = kP;
    talonFxConfig.Slot0.kI = kI;
    talonFxConfig.Slot0.kD = kD;
    talonFxConfig.Slot0.kV = kV;
    talonFxConfig.Slot0.kS = kS;

    tryUntilOk(5, () -> motor.getConfigurator().apply(talonFxConfig, 0.25));
  } // End FlywheelIOTalonFX Constructor

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    var signalRefreshStatus =
        BaseStatusSignal.refreshAll(
            motor.getVelocity(),
            motor.getMotorVoltage(),
            motor.getSupplyCurrent());
    inputs.motorConnected = signalRefreshStatus.equals(StatusCode.OK);
    double motorShaftRps = motor.getVelocity().getValueAsDouble();
    inputs.velocityRadsPerSec = Units.rotationsToRadians(motorShaftRps) / kGearRatio;
    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.supplyCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
  } // End updateInputs

  @Override
  public void setTargetVelocity(double targetVelocityRadsPerSec) {
    double targetRps = targetVelocityRadsPerSec / (2.0 * Math.PI);
    double motorRps = targetRps * kGearRatio;
    motor.setControl(velocityVoltageRequest.withVelocity(motorRps));
  } // End setTargetVelocity

  @Override
  public void stop() {
    motor.setControl(new NeutralOut());
  } // End stop
}
