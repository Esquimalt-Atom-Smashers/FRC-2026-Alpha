package frc.robot.subsystems.agitator;

import static frc.robot.subsystems.agitator.AgitatorConstants.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Agitator subsystem: intake-to-shooter transfer, one motor, velocity controlled (run or coast). */
public class Agitator extends SubsystemBase {

  private final AgitatorIO agitatorIO;
  private final AgitatorIO.AgitatorIOInputs agitatorInputs = new AgitatorIO.AgitatorIOInputs();

  private boolean isRunning = false;

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
    Logger.recordOutput("Agitator/VelocityRpm", getVelocityRpm());

    if (DriverStation.isDisabled()) {
      agitatorIO.stop();
      return;
    }

    if (isRunning) {
      agitatorIO.setTargetVelocity(kTargetVelocityRadsPerSec);
    } else {
      agitatorIO.stop();
    }
  } // End periodic

  /** Run the agitator at the configured target velocity. */
  public void run() {
    isRunning = true;
  } // End run

  /** Stop the agitator (coast). */
  public void stop() {
    isRunning = false;
  } // End stop

  /** Whether the agitator is currently commanded to run. */
  public boolean isRunning() {
    return isRunning;
  } // End isRunning

  /** Current velocity (rad/s). */
  public double getVelocityRadsPerSec() {
    return agitatorInputs.velocityRadsPerSec;
  } // End getVelocityRadsPerSec

  /** Current velocity (RPM). */
  public double getVelocityRpm() {
    return Units.radiansPerSecondToRotationsPerMinute(agitatorInputs.velocityRadsPerSec);
  } // End getVelocityRpm
}
