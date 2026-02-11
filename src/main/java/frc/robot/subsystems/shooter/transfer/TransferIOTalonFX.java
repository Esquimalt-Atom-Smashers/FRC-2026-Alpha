package frc.robot.subsystems.shooter.transfer;

import static frc.robot.subsystems.shooter.transfer.TransferConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusCode;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.generated.TunerConstants;

/** Transfer IO using Talon FX (Kraken) velocity control and REV Color Sensor V3. */
public class TransferIOTalonFX implements TransferIO {

  private final TalonFX motor;
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);
  private final ColorSensorV3 colorSensor;

  public TransferIOTalonFX() {
    motor = new TalonFX(kMotorId, TunerConstants.kCANBus);
    colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

    var talonFxConfig = new TalonFXConfiguration();
    talonFxConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    talonFxConfig.MotorOutput.Inverted =
        kMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    talonFxConfig.Slot0.kP = kP;
    talonFxConfig.Slot0.kI = kI;
    talonFxConfig.Slot0.kD = kD;
    talonFxConfig.Slot0.kV = kV;
    talonFxConfig.Slot0.kS = kS;

    tryUntilOk(5, () -> motor.getConfigurator().apply(talonFxConfig, 0.25));
  }

  @Override
  public void updateInputs(TransferIOInputs inputs) {
    var signalRefreshStatus =
        BaseStatusSignal.refreshAll(
            motor.getVelocity(),
            motor.getMotorVoltage(),
            motor.getSupplyCurrent());
    inputs.motorConnected = signalRefreshStatus.equals(StatusCode.OK);
    double motorShaftRps = motor.getVelocity().getValueAsDouble();
    inputs.velocityRadsPerSec =
        (kMotorInverted ? -1.0 : 1.0)
            * Units.rotationsToRadians(motorShaftRps) / kGearRatio;
    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.supplyCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
    inputs.colorSensorTripped = colorSensor.getProximity() >= kColorSensorProximityThreshold;
  }

  @Override
  public void setTargetVelocity(double targetVelocityRadsPerSec) {
    double targetRps = targetVelocityRadsPerSec / (2.0 * Math.PI);
    double motorRps = targetRps * kGearRatio;
    motor.setControl(velocityVoltageRequest.withVelocity(motorRps));
  }

  @Override
  public void stop() {
    motor.setControl(new NeutralOut());
  }
}
