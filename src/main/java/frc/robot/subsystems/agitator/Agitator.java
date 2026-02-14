package frc.robot.subsystems.agitator;

import static frc.robot.subsystems.agitator.AgitatorConstants.*;

import edu.wpi.first.math.util.Units;
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
  private double targetVelocityRadsPerSec = kStagingVelocityRadsPerSec;

  public Agitator(AgitatorIO io) {
    agitatorIO = io;
  } // End Agitator Constructor

  @Override
  public void periodic() {
    agitatorIO.updateInputs(agitatorInputs);
    Logger.recordOutput("Agitator/Inputs/MotorConnected", agitatorInputs.motorConnected);
    Logger.recordOutput("Agitator/Inputs/VelocityRadsPerSec", agitatorInputs.velocityRadsPerSec);
    Logger.recordOutput("Agitator/Inputs/AppliedVolts", agitatorInputs.appliedVolts);
    Logger.recordOutput("Agitator/Inputs/SupplyCurrentAmps", agitatorInputs.supplyCurrentAmps);
    Logger.recordOutput("Agitator/Mode", mode.name());
    Logger.recordOutput("Agitator/VelocityRpm", getVelocityRpm());

    if (DriverStation.isDisabled()) {
      agitatorIO.stop();
      return;
    }

    switch (mode) {
      case IDLE:
        agitatorIO.stop();
        break;
      case STAGING:
        agitatorIO.setTargetVelocity(targetVelocityRadsPerSec);
        break;
      case SHOOTING:
        agitatorIO.setTargetVelocity(targetVelocityRadsPerSec);
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

  /** Set mode to staging (slow velocity). Transitions to IDLE when Transfer goes idle. */
  public void setStagingMode() {
    mode = Mode.STAGING;
    targetVelocityRadsPerSec = kStagingVelocityRadsPerSec;
  } // End setStagingMode

  /** Set mode to shooting (high velocity). */
  public void setShootingMode() {
    mode = Mode.SHOOTING;
    targetVelocityRadsPerSec = kShootingVelocityRadsPerSec;
  } // End setShootingMode

  /** Set the target velocity (rad/s) used when in STAGING or SHOOTING. */
  public void setTargetVelocityRadsPerSec(double radsPerSec) {
    targetVelocityRadsPerSec = radsPerSec;
  } // End setTargetVelocityRadsPerSec

  /** Get the current target velocity (rad/s). */
  public double getTargetVelocityRadsPerSec() {
    return mode == Mode.IDLE ? 0.0 : targetVelocityRadsPerSec;
  } // End getTargetVelocityRadsPerSec

  /** Get the current target velocity (RPM). */
  public double getTargetVelocityRpm() {
    return Units.radiansPerSecondToRotationsPerMinute(getTargetVelocityRadsPerSec());
  } // End getTargetVelocityRpm

  /** Current mode. */
  public Mode getMode() {
    return mode;
  } // End getMode

  /** Current velocity (rad/s). */
  public double getVelocityRadsPerSec() {
    return agitatorInputs.velocityRadsPerSec;
  } // End getVelocityRadsPerSec

  /** Current velocity (RPM). */
  public double getVelocityRpm() {
    return Units.radiansPerSecondToRotationsPerMinute(agitatorInputs.velocityRadsPerSec);
  } // End getVelocityRpm
}
