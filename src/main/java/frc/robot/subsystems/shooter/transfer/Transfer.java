package frc.robot.subsystems.shooter.transfer;

import static frc.robot.subsystems.shooter.transfer.TransferConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Transfer subsystem: staging (low voltage, stop when sensor tripped) or shooting (high voltage). */
public class Transfer extends SubsystemBase {

  /** Transfer mode: idle, staging (slow pre-load), or shooting. */
  public enum Mode {
    IDLE,
    STAGING,
    SHOOTING
  }

  private final TransferIO transferIO;
  private final TransferIO.TransferIOInputs transferInputs = new TransferIO.TransferIOInputs();

  private Mode mode = Mode.IDLE;
  private boolean ballStaged = false;
  private double targetVoltage = kIdleVoltage;

  public Transfer(TransferIO io) {
    transferIO = io;
  } // End Transfer Constructor

  @Override
  public void periodic() {
    transferIO.updateInputs(transferInputs);
    Logger.recordOutput("Transfer/Inputs/MotorConnected", transferInputs.motorConnected);
    Logger.recordOutput("Transfer/Inputs/AppliedVolts", transferInputs.appliedVolts);
    Logger.recordOutput("Transfer/Inputs/SupplyCurrentAmps", transferInputs.supplyCurrentAmps);
    Logger.recordOutput("Transfer/Inputs/ColorSensorTripped", transferInputs.colorSensorTripped);
    Logger.recordOutput("Transfer/Mode", mode.name());
    Logger.recordOutput("Transfer/BallStaged", ballStaged);

    if (DriverStation.isDisabled()) {
      transferIO.stop();
      return;
    }

    switch (mode) {
      case IDLE:
        transferIO.stop();
        break;
      case STAGING:
        if (transferInputs.colorSensorTripped) {
          transferIO.stop();
          ballStaged = true;
          mode = Mode.IDLE;
        } else {
          transferIO.setVoltage(targetVoltage);
        }
        break;
      case SHOOTING:
        transferIO.setVoltage(targetVoltage);
        break;
      default:
        transferIO.stop();
        break;
    }
  } // End periodic

  /** Set mode to staging (low voltage; stop when colour sensor tripped). */
  public void setStagingMode() {
    mode = Mode.STAGING;
    targetVoltage = kStagingVoltage;
  } // End setStagingMode

  /** Set mode to shooting (high voltage); clears ballStaged. */
  public void setShootingMode() {
    mode = Mode.SHOOTING;
    ballStaged = false;
    targetVoltage = kShootingVoltage;
  } // End setShootingMode

  /** Set the target voltage (V) used when in STAGING or SHOOTING. */
  public void setTargetVoltage(double volts) {
    targetVoltage = volts;
  } // End setTargetVoltage

  /** Get the current target voltage (V). */
  public double getTargetVoltage() {
    return mode == Mode.IDLE ? 0.0 : targetVoltage;
  } // End getTargetVoltage

  /** Set mode to idle (motor stopped). */
  public void setIdleMode() {
    mode = Mode.IDLE;
  } // End setIdleMode

  /** Current mode. */
  public Mode getMode() {
    return mode;
  } // End getMode

  /** True when staging and colour sensor was tripped (ball at transfer). */
  public boolean isBallStaged() {
    return ballStaged;
  } // End isBallStaged
}
