package frc.robot.subsystems.agitator;

import static frc.robot.subsystems.agitator.AgitatorConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Agitator subsystem: storage-to-shooter transfer */
public class Agitator extends SubsystemBase {

  /** Agitator mode: idle, staging (slow pre-load), or shooting. */
  public enum Mode {
    IDLE,
    STAGING,
    SHOOTING
  }

  private final AgitatorIO agitatorIO;
  private final AgitatorIO.AgitatorIOInputs agitatorInputs = new AgitatorIO.AgitatorIOInputs();

  private Mode mode = Mode.IDLE;
  private double targetVoltage = kIdleVoltage;

  public Agitator(AgitatorIO io) {
    agitatorIO = io;
  } // End Agitator Constructor

  @Override
  public void periodic() {
    agitatorIO.updateInputs(agitatorInputs);
    Logger.recordOutput("Agitator/Inputs/MotorConnected", agitatorInputs.motorConnected);
    Logger.recordOutput("Agitator/Inputs/AppliedVolts", agitatorInputs.appliedVolts);
    Logger.recordOutput("Agitator/Inputs/SupplyCurrentAmps", agitatorInputs.supplyCurrentAmps);
    Logger.recordOutput("Agitator/Mode", mode.name());

    if (DriverStation.isDisabled()) {
      agitatorIO.stop();
      return;
    }

    switch (mode) {
      case IDLE:
        agitatorIO.stop();
        break;
      case STAGING:
        agitatorIO.setVoltage(targetVoltage);
        break;
      case SHOOTING:
        agitatorIO.setVoltage(targetVoltage);
        break;
      default:
        agitatorIO.stop();
        break;
    }
  } // End periodic

  /** Set mode to idle (motor stopped). */
  public void setIdleMode() {
    mode = Mode.IDLE;
  } // End setIdleMode

  /** Set mode to staging (low voltage). */
  public void setStagingMode() {
    mode = Mode.STAGING;
    targetVoltage = kStagingVoltage;
  } // End setStagingMode

  /** Set mode to shooting (high voltage). */
  public void setShootingMode() {
    mode = Mode.SHOOTING;
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

  /** Current mode. */
  public Mode getMode() {
    return mode;
  } // End getMode
}
