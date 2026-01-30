package frc.robot.subsystems.shooter.flywheel;

import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Flywheel subsystem: one motor with onboard velocity control; state machine Idle / Charging / AtSpeed. */
public class Flywheel extends SubsystemBase {

  /** Flywheel state: Idle (low velocity), Charging (ramping to target), AtSpeed (ready to shoot). */
  public enum FlywheelState {
    IDLE,
    CHARGING,
    AT_SPEED
  }

  private final FlywheelIO flywheelIO;
  private final FlywheelIO.FlywheelIOInputs flywheelInputs = new FlywheelIO.FlywheelIOInputs();

  private FlywheelState state = FlywheelState.IDLE;

  public Flywheel(FlywheelIO io) {
    flywheelIO = io;
  } // End Flywheel Constructor

  @Override
  public void periodic() {
    // Update the flywheel inputs and record the values
    flywheelIO.updateInputs(flywheelInputs);
    Logger.recordOutput("Flywheel/Inputs/MotorConnected", flywheelInputs.motorConnected);
    Logger.recordOutput("Flywheel/Inputs/VelocityRadsPerSec", flywheelInputs.velocityRadsPerSec);
    Logger.recordOutput("Flywheel/Inputs/AppliedVolts", flywheelInputs.appliedVolts);
    Logger.recordOutput("Flywheel/Inputs/SupplyCurrentAmps", flywheelInputs.supplyCurrentAmps);
    Logger.recordOutput("Flywheel/VelocityRpm", getVelocityRpm());
    Logger.recordOutput("Flywheel/TargetVelocityRpm", getTargetVelocityRpm());
    Logger.recordOutput("Flywheel/State", state.name());

    if (DriverStation.isDisabled()) {
      flywheelIO.stop();
      return;
    }

    // Auto-transition Charging → AtSpeed when at target velocity
    if (state == FlywheelState.CHARGING && atTargetVelocity()) {
      state = FlywheelState.AT_SPEED;
    }

    // If in AtSpeed but velocity fell below/outside tolerance, go back to Charging to re-ramp
    if (state == FlywheelState.AT_SPEED && !atTargetVelocity()) {
      state = FlywheelState.CHARGING;
    }

    double targetVelocityRadsPerSec =
        state == FlywheelState.IDLE ? kIdleVelocityRadsPerSec : kDefaultTargetVelocityRadsPerSec;
    flywheelIO.setTargetVelocity(targetVelocityRadsPerSec);
  } // End periodic

  /** Set the flywheel state. */
  public void setState(FlywheelState newState) {
    state = newState;
  } // End setState

  /** Get the current flywheel state. */
  public FlywheelState getState() {
    return state;
  } // End getState

  /** Get the current target velocity (rad/s) for the current state. */
  public double getTargetVelocityRadsPerSec() {
    return state == FlywheelState.IDLE ? kIdleVelocityRadsPerSec : kDefaultTargetVelocityRadsPerSec;
  } // End getTargetVelocityRadsPerSec

  /** Get the current target velocity (RPM). */
  public double getTargetVelocityRpm() {
    return Units.radiansPerSecondToRotationsPerMinute(getTargetVelocityRadsPerSec());
  } // End getTargetVelocityRpm

  /** Get the current velocity (rad/s). */
  public double getVelocityRadsPerSec() {
    return flywheelInputs.velocityRadsPerSec;
  } // End getVelocityRadsPerSec

  /** Get the current velocity (RPM). */
  public double getVelocityRpm() {
    return Units.radiansPerSecondToRotationsPerMinute(flywheelInputs.velocityRadsPerSec);
  } // End getVelocityRpm

  /** Whether the flywheel is at target velocity within tolerance. */
  public boolean atTargetVelocity() {
    double currentVelocityRadsPerSec = flywheelInputs.velocityRadsPerSec;
    double targetVelocityRadsPerSec = getTargetVelocityRadsPerSec();
    return Math.abs(currentVelocityRadsPerSec - targetVelocityRadsPerSec)
        <= kAtTargetVelocityToleranceRadsPerSec;
  } // End atTargetVelocity
}
