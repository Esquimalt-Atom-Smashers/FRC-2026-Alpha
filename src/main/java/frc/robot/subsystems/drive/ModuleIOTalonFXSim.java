// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.PhoenixUtil;
import java.util.Arrays;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

/**
 * Physics sim implementation of module IO using GenericMotorController for direct voltage control.
 * This follows the Hardware Abstraction approach from MapleSim docs, calculating voltage directly
 * instead of relying on TalonFX controller's velocity control loop.
 */
public class ModuleIOTalonFXSim extends ModuleIOTalonFX {
  private final SwerveModuleSimulation simulation;
  private final SimulatedMotorController.GenericMotorController driveMotor;
  private final SimulatedMotorController.GenericMotorController steerMotor;
  
  // Store constants for access in setDriveVelocity
  private final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> moduleConstants;

  // PID controllers for velocity/position control
  private final PIDController driveController;
  private final PIDController turnController;
  
  // State tracking
  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;
  
  // Velocity tracking (for PID control)
  private double previousDrivePositionRad = 0.0;
  private double previousTurnPositionRad = 0.0;
  private double driveVelocityRadPerSec = 0.0;
  private double turnVelocityRadPerSec = 0.0;
  private double previousTimestamp = 0.0;

  public ModuleIOTalonFXSim(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants,
      SwerveModuleSimulation simulation) {
        
    super(PhoenixUtil.regulateModuleConstantForSimulation(constants));

    this.moduleConstants = constants;
    this.simulation = simulation;
    
    // Use GenericMotorController for direct voltage control (Hardware Abstraction approach)
    this.driveMotor = simulation.useGenericMotorControllerForDrive()
        .withCurrentLimit(Amps.of(60));
    this.steerMotor = simulation.useGenericControllerForSteer()
        .withCurrentLimit(Amps.of(20));

    // Get regulated constants for PID gains
    var regulatedConstants = PhoenixUtil.regulateModuleConstantForSimulation(constants);
    
    // Create PID controllers using regulated gains
    this.driveController = new PIDController(
        regulatedConstants.DriveMotorGains.kP, 
        0, 
        regulatedConstants.DriveMotorGains.kD);
    
    this.turnController = new PIDController(
        regulatedConstants.SteerMotorGains.kP,
        0,
        regulatedConstants.SteerMotorGains.kD);
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Get current positions from simulation
    double drivePositionRad = simulation.getDriveWheelFinalPosition().in(Radians);
    Rotation2d turnPosition = simulation.getSteerAbsoluteFacing();
    double turnPositionRad = turnPosition.getRadians();
    
    // Calculate velocities from position changes
    double currentTimestamp = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    double dt = currentTimestamp - previousTimestamp;
    if (dt > 0 && previousTimestamp > 0) {
      // Calculate velocity from position change
      driveVelocityRadPerSec = (drivePositionRad - previousDrivePositionRad) / dt;
      
      // Handle wrap-around for turn position
      double turnDelta = turnPositionRad - previousTurnPositionRad;
      if (turnDelta > Math.PI) turnDelta -= 2 * Math.PI;
      if (turnDelta < -Math.PI) turnDelta += 2 * Math.PI;
      turnVelocityRadPerSec = turnDelta / dt;
    }
    previousDrivePositionRad = drivePositionRad;
    previousTurnPositionRad = turnPositionRad;
    previousTimestamp = currentTimestamp;
    
    // Run closed-loop control
    if (driveClosedLoop) {
      driveAppliedVolts = driveFFVolts + driveController.calculate(driveVelocityRadPerSec);
      driveAppliedVolts = MathUtil.clamp(driveAppliedVolts, -12.0, 12.0);
      driveMotor.requestVoltage(Volts.of(driveAppliedVolts));
    } else {
      driveController.reset();
      driveMotor.requestVoltage(Volts.of(driveAppliedVolts));
    }
    
    if (turnClosedLoop) {
      turnAppliedVolts = turnController.calculate(turnPositionRad);
      turnAppliedVolts = MathUtil.clamp(turnAppliedVolts, -12.0, 12.0);
      steerMotor.requestVoltage(Volts.of(turnAppliedVolts));
    } else {
      turnController.reset();
      steerMotor.requestVoltage(Volts.of(turnAppliedVolts));
    }

    // Update inputs with simulation values
    inputs.driveConnected = true;
    inputs.drivePositionRad = drivePositionRad;
    inputs.driveVelocityRadPerSec = driveVelocityRadPerSec;
    inputs.driveAppliedVolts = driveAppliedVolts;
    // Current not directly available from GenericMotorController, estimate from voltage
    inputs.driveCurrentAmps = Math.abs(driveAppliedVolts / 12.0 * 60.0); // Rough estimate

    inputs.turnConnected = true;
    inputs.turnEncoderConnected = true;
    inputs.turnAbsolutePosition = turnPosition;
    inputs.turnPosition = turnPosition;
    inputs.turnVelocityRadPerSec = turnVelocityRadPerSec;
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(turnAppliedVolts / 12.0 * 20.0); // Rough estimate

    // Update odometry inputs
    inputs.odometryTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();
    inputs.odometryDrivePositionsRad = Arrays.stream(simulation.getCachedDriveWheelFinalPositions())
        .mapToDouble(angle -> angle.in(Radians))
        .toArray();
    inputs.odometryTurnPositions = simulation.getCachedSteerAbsolutePositions();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = output; // output is already in voltage units (matches ModuleIOSim)
    driveMotor.requestVoltage(Volts.of(driveAppliedVolts));
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnClosedLoop = false;
    turnAppliedVolts = output; // output is already in voltage units (matches ModuleIOSim)
    steerMotor.requestVoltage(Volts.of(turnAppliedVolts));
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    driveClosedLoop = true;
    
    // Get regulated constants for feedforward calculation
    var regulatedConstants = PhoenixUtil.regulateModuleConstantForSimulation(
        (SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>) moduleConstants);
    
    // Calculate feedforward voltage (KS * sign + KV * velocity)
    double ks = regulatedConstants.DriveMotorGains.kS;
    double kv = regulatedConstants.DriveMotorGains.kV;
    driveFFVolts = ks * Math.signum(velocityRadPerSec) + kv * velocityRadPerSec;
    
    driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnClosedLoop = true;
    turnController.setSetpoint(rotation.getRadians());
  }
}
