// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.DashboardNames;

public class Flywheel extends SubsystemBase
{
  public static final int LeadMotorId = 21;
  public static final int FollowMotorId = 22;

  // Back to 1:1 for long-range shot.
  public static final double gearRatio = 1.0;

  public static final double wheelDiameterMeters = 3.0 * 0.0254;
  public static final double turnsPerMeter = 1.0 / (wheelDiameterMeters * Math.PI);
  public static final double ampsPerNewton = 10.0; // TODO: Get amps per newton
  public static final double currentLimit = 60.0;

  private final TalonFX leadMotor;
  private final TalonFX followMotor;
  private final StatusSignal<AngularVelocity> velocitySig;
  private final StatusSignal<Current> currentSig;
  private final StatusSignal<Current> followerCurrentSig;
  private final VelocityVoltage flywheelVelocityVoltage = new VelocityVoltage(0).withSlot(0);
  private final SlewRateLimiter limiter = new SlewRateLimiter(20.0);

  private boolean hasCommand = false;
  private double targetVelocity = 0.0;

  private double velocity = 0.0;
  private double current = 0.0;
  private double followerVelocity = 0.0;
  private double followerCurrent = 0.0;

  public Flywheel()
  {
    setName("Flywheel");

    leadMotor = new TalonFX(LeadMotorId, CANBus.roboRIO());
    followMotor = new TalonFX(FollowMotorId, CANBus.roboRIO());
    velocitySig = leadMotor.getVelocity();
    currentSig = leadMotor.getTorqueCurrent();
    followerCurrentSig = followMotor.getTorqueCurrent();

    boolean hardwareConfigured = configureHardware();
    if (!hardwareConfigured) {
      System.err.println("Flywheel failed to conifgure:");
    }
    SmartDashboard.putBoolean(DashboardNames.FLYWHEEL_HW_CONFIGURED.getKey(), hardwareConfigured);
  }

  private boolean configureHardware()
  {
    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.TorqueCurrent.PeakForwardTorqueCurrent = 10.0;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -10.0;

    configs.Voltage.PeakForwardVoltage = 10.0;
    configs.Voltage.PeakReverseVoltage = -10.0;

    configs.CurrentLimits.SupplyCurrentLimit = currentLimit;
    configs.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Slot 0 for the velocity control loop:
    configs.Slot0.kV = 0.12; // Motor kV plus Boost for friction.
    configs.Slot0.kP = 0.35;
    configs.Slot0.kI = 0.0;
    configs.Slot0.kD = 0.01;
    configs.Slot0.kA = 0.0;
    configs.Slot0.kS = 0.02;

    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    var status = leadMotor.getConfigurator().apply(configs, 1.0);
    if (!status.isOK()) {
      System.err.println("Flywheel lead motor configfuration failed.");
      return false;
    }
    leadMotor.setNeutralMode(NeutralModeValue.Coast, 1.0);

    TalonFXConfiguration followerConfigs = new TalonFXConfiguration();
    followerConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // change this if directions are the same.
    followerConfigs.CurrentLimits.SupplyCurrentLimit = currentLimit;
    followerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    followerConfigs.Voltage.PeakForwardVoltage = 10.0;
    followerConfigs.Voltage.PeakReverseVoltage = -10.0;

    status = followMotor.getConfigurator().apply(followerConfigs, 1.0);
    if (!status.isOK()) {
      System.err.println("Flywheel follower motor configfuration failed.");
      return false;
    }

    return true;
  }

  // Command the flywheel to spin at the given velocity, meters/second of wheel surface speed.
  public void setVelocity(double metersPerSecond)
  {
    hasCommand = true;
    targetVelocity = metersPerSecond;
  }

  public void stop()
  {
    hasCommand = false;
  }

  public double getTargetVelocity()
  {
    return hasCommand ? targetVelocity : 0.0;
  }

  public double getVelocity()
  {
    return velocity;
  }

  public double getCurrent()
  {
    return current;
  }

  public double getFollowerVelocity()
  {
    return followerVelocity;
  }

  public double getFollowerCurrent()
  {
    return followerCurrent;
  }

  @Override
  public void periodic()
  {
    velocitySig.refresh();
    currentSig.refresh();
    followerCurrentSig.refresh();

    velocity = velocitySig.getValueAsDouble() / (turnsPerMeter * gearRatio);
    // No separate follower velocity signal exists; matches C++ reusing the lead motor's velocity here.
    followerVelocity = velocitySig.getValueAsDouble() / (turnsPerMeter * gearRatio);
    current = currentSig.getValueAsDouble();
    followerCurrent = followerCurrentSig.getValueAsDouble();

    if (hasCommand) {
      // Compute a rate-limited velocity:
      double limitedVelocity = limiter.calculate(targetVelocity);
      double motorVelocity = limitedVelocity * turnsPerMeter * gearRatio;

      // Send commands to motors:
      leadMotor.setControl(flywheelVelocityVoltage.withVelocity(motorVelocity));
      followMotor.setControl(new Follower(leadMotor.getDeviceID(), MotorAlignmentValue.Aligned));
    } else {
      // Send commands to motors:
      leadMotor.setControl(new NeutralOut());
      followMotor.setControl(new NeutralOut());
      // Reset the limiter:
      limiter.reset(velocity); // Be ready to re-start from wherever we actually are...
    }

    SmartDashboard.putNumber(DashboardNames.FLYWHEEL_ANGULAR_VELOCITY.getKey(), 60.0 * velocitySig.getValueAsDouble());
    SmartDashboard.putNumber(DashboardNames.FLYWHEEL_TARGET_VELOCITY.getKey(), limiter.lastValue());
    SmartDashboard.putNumber(DashboardNames.FLYWHEEL_VELOCITY.getKey(), velocity);
    SmartDashboard.putNumber(DashboardNames.FLYWHEEL_CURRENT.getKey(), current);
    SmartDashboard.putNumber(DashboardNames.FLYWHEEL_FOLLOWER_CURRENT.getKey(), followerCurrent);
  }
}
