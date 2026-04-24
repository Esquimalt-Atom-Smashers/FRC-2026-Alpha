package frc.robot.subsystems.hang;

import static frc.robot.subsystems.hang.HangConstants.kStoredPositionMeters;

import edu.wpi.first.math.MathUtil;

/** Hang IO for simulation; rate-limited setpoint following. */
public class HangIOSim implements HangIO {

  public enum States {
    TARGETINGPOSITION,
    TARGETINGVOLTAGE
  } // End State enum

  private static final double kLoopPeriodSecs = 0.02;
  private static final double kMaxMetersPerSec = 0.04064;

  private double targetPositionMeters = 0.0;
  private double currentPositionMeters = 0.0;
  private double targetVoltageMeters = 0.0;
  private boolean isStopped = false;
  
  private States currentState = States.TARGETINGPOSITION;

  @Override
  public void updateInputs(HangIOInputs inputs) {
    if (!isStopped) {
      double errorMeters = targetPositionMeters - currentPositionMeters;
      double maxStepMeters = kMaxMetersPerSec * kLoopPeriodSecs;
      double stepMeters = 0;
      switch (currentState) {
        case TARGETINGPOSITION:
          stepMeters = MathUtil.clamp(errorMeters, -maxStepMeters, maxStepMeters);
          break;
        case TARGETINGVOLTAGE:
          stepMeters = targetVoltageMeters;
          break;
        default:
          break;
      }
      currentPositionMeters += stepMeters;
      inputs.velocityMetersPerSec = stepMeters / kLoopPeriodSecs;
    } else {
      inputs.velocityMetersPerSec = 0.0;
    }

    inputs.motorConnected = true;
    inputs.positionMeters = currentPositionMeters;
    inputs.appliedVolts = 0.0;
    inputs.supplyCurrentAmps = 0.0;
  } // End updateInputs

  @Override
  public void setTargetPosition(double targetMeters) {
    currentState = States.TARGETINGPOSITION;
    targetPositionMeters = targetMeters;
    isStopped = false;
  } // End setTargetPosition

  @Override
  public void setVoltage(double targetVoltage){
    currentState = States.TARGETINGVOLTAGE;
    targetVoltageMeters = targetVoltage; // doesn't actually convert voltage to m/s
    isStopped = false;
  }

  @Override
  public boolean isCalibrated(){
    boolean fakeSensor = (currentPositionMeters <= 0);
    return fakeSensor;
  }

  @Override
  public void resetEncoders() {
    currentPositionMeters = kStoredPositionMeters;
  } // End resetEncoders

  @Override
  public void stop() {
    isStopped = true;
  } // End stop
}
