package frc.robot.subsystems.shooter.transfer;

import static frc.robot.subsystems.shooter.transfer.TransferConstants.*;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.REVLibError;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;

/** Transfer IO using SPARK MAX (NEO 550) velocity control and REV Color Sensor V3. */
public class TransferIOSparkMax implements TransferIO {

  private final SparkMax motor;
  private final SparkClosedLoopController closedLoopController;
  private final RelativeEncoder encoder;
  private final ColorSensorV3 colorSensor;

  public TransferIOSparkMax() {
    motor = new SparkMax(kMotorId, SparkLowLevel.MotorType.kBrushless);
    closedLoopController = motor.getClosedLoopController();
    encoder = motor.getEncoder();
    colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

    var sparkMaxConfig = new SparkMaxConfig();
    sparkMaxConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
    sparkMaxConfig.inverted(kMotorInverted);
    sparkMaxConfig
        .encoder
        .positionConversionFactor(1.0 / kGearRatio)
        .velocityConversionFactor(1.0 / kGearRatio);
    sparkMaxConfig.closedLoop.p(kP).i(kI).d(kD);
    motor.configure(sparkMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(TransferIOInputs inputs) {
    inputs.motorConnected = motor.getLastError() == REVLibError.kOk;
    double rawRpm = encoder.getVelocity();
    inputs.velocityRadsPerSec =
        (kMotorInverted ? -1.0 : 1.0)
            * Units.rotationsPerMinuteToRadiansPerSecond(rawRpm);
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.supplyCurrentAmps = motor.getOutputCurrent();
    inputs.colorSensorTripped = colorSensor.getProximity() >= kColorSensorProximityThreshold;
  }

  @Override
  public void setTargetVelocity(double targetVelocityRadsPerSec) {
    double setpointRpm =
        Units.radiansPerSecondToRotationsPerMinute(targetVelocityRadsPerSec);
    closedLoopController.setSetpoint(setpointRpm, SparkBase.ControlType.kVelocity);
  }

  @Override
  public void stop() {
    motor.set(0.0);
  }
}
