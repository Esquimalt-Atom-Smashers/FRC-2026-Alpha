package frc.robot.subsystems.shooter.turret;

import static frc.robot.subsystems.shooter.turret.TurretConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Turret subsystem: one motor with PID position control, aimed at a goal angle. */
public class Turret extends SubsystemBase {

  private final TurretIO turretIO; // Hardware Abstraction IO Interface for the turret motors
  private final TurretIO.TurretIOInputs turretInputs = new TurretIO.TurretIOInputs(); // Data Container for the turret values
  private final PIDController pid = new PIDController(kP, kI, kD);

  private Rotation2d goalAngle = Rotation2d.kZero;

  public Turret(TurretIO io) {
    this.turretIO = io;
    pid.setTolerance(Units.degreesToRadians(2.0));
  }

  @Override
  public void periodic() {
    turretIO.updateInputs(turretInputs);

    Logger.recordOutput("Turret/Inputs/MotorConnected", turretInputs.motorConnected);
    Logger.recordOutput("Turret/Inputs/PositionRads", turretInputs.positionRads);
    Logger.recordOutput("Turret/Inputs/VelocityRadsPerSec", turretInputs.velocityRadsPerSec);
    Logger.recordOutput("Turret/Inputs/AppliedVolts", turretInputs.appliedVolts);
    Logger.recordOutput("Turret/Inputs/SupplyCurrentAmps", turretInputs.supplyCurrentAmps);
    Logger.recordOutput("Turret/PositionDegrees", getPosition().getDegrees());
    Logger.recordOutput("Turret/GoalDegrees", getGoalAngle().getDegrees());

    if (DriverStation.isDisabled()) {
      turretIO.setVoltage(0.0);
      return;
    }

    double goalRad = MathUtil.clamp(goalAngle.getRadians(), kMinAngleRad, kMaxAngleRad);
    double positionRad = turretInputs.positionRads + kEncoderZeroOffsetRad;

    double output = pid.calculate(positionRad, goalRad);
    output = MathUtil.clamp(output, -kMaxVoltage, kMaxVoltage);

    turretIO.setVoltage(output);

    // TODO: Better to use kPosition control in the motor controllers, so that the PID control is done on the devices instead of the code
  }

  /** Set the goal angle (turret frame). Clamped to min/max in periodic. */
  public void setGoalAngle(Rotation2d angle) {
    this.goalAngle = angle;
  }

  /** Get the current goal angle. */
  public Rotation2d getGoalAngle() {
    return goalAngle;
  }

  /** Get the current turret position (robot frame: 0 = forward). */
  public Rotation2d getPosition() {
    return Rotation2d.fromRadians(turretInputs.positionRads + kEncoderZeroOffsetRad);
  }

  /** Whether the turret is at the goal within tolerance. */
  public boolean atGoal() {
    return pid.atSetpoint();
  }
}
