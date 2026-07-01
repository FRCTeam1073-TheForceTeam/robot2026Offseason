// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Collector extends SubsystemBase
{
  public static final int MotorId = 20;

  public static final double wheelDiameterMeters = 1.25 * 0.0254;
  public static final double gearRatio = 2.0;
  public static final double turnsPerMeter = 1.0 / (wheelDiameterMeters * Math.PI);
  public static final double ampsPerNewton = 10.0;
  public static final double currentLimit = 50.0;

  private final TalonFX motor;
  private final StatusSignal<AngularVelocity> velocitySig;
  private final StatusSignal<Current> currentSig;
  private final VelocityVoltage commandVelocityVoltage = new VelocityVoltage(0).withSlot(0);
  private final SlewRateLimiter limiter = new SlewRateLimiter(10.0);

  private boolean hasCommand = false;
  private double targetVelocity = 0.0;

  private double velocity = 0.0;
  private double force = 0.0;

  public Collector()
  {
    setName("Collector");

    motor = new TalonFX(MotorId, new CANBus("rio"));
    velocitySig = motor.getVelocity();
    currentSig = motor.getTorqueCurrent();

    boolean hardwareConfigured = configureHardware();
    if (!hardwareConfigured) {
      System.err.println("Collector: Hardware Failed To Configure!");
    }
    SmartDashboard.putBoolean("Collector/Collector - hardware_configured", hardwareConfigured);
  }

  private boolean configureHardware()
  {
    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.TorqueCurrent.PeakForwardTorqueCurrent = 10.0;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -10.0;

    configs.Voltage.PeakForwardVoltage = 9.0;
    configs.Voltage.PeakReverseVoltage = -9.0;

    configs.CurrentLimits.SupplyCurrentLimit = currentLimit;
    configs.CurrentLimits.SupplyCurrentLimitEnable = true;

    configs.Slot0.kV = 0.123;
    configs.Slot0.kP = 0.3;
    configs.Slot0.kI = 0.0;
    configs.Slot0.kD = 0.0;
    configs.Slot0.kA = 0.0;
    configs.Slot0.kS = 0.02;

    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    var status = motor.getConfigurator().apply(configs, 1.0);
    if (!status.isOK()) {
      System.err.println("Collector: config failed to config!");
      return false;
    }

    status = motor.setNeutralMode(NeutralModeValue.Brake, 1.0);
    if (!status.isOK()) {
      System.err.println("Collector: neutral mode failed to config :(!");
      return false;
    }

    return true;
  }

  // Command the collector to spin at the given velocity, meters/second of wheel surface speed.
  public void setVelocity(double metersPerSecond)
  {
    hasCommand = true;
    targetVelocity = metersPerSecond;
  }

  public void stop()
  {
    hasCommand = false;
  }

  public double getVelocity()
  {
    return velocity;
  }

  public double getForce()
  {
    return force;
  }

  @Override
  public void periodic()
  {
    velocitySig.refresh();
    currentSig.refresh();

    force = currentSig.getValueAsDouble() / ampsPerNewton;
    velocity = velocitySig.getValueAsDouble() / (turnsPerMeter * gearRatio);

    if (hasCommand) {
      double limitedVel = limiter.calculate(targetVelocity);
      double motorVel = limitedVel * turnsPerMeter * gearRatio;
      motor.setControl(commandVelocityVoltage.withVelocity(motorVel));
    } else {
      motor.setControl(new NeutralOut());
      limiter.reset(0.0);
    }

    SmartDashboard.putNumber("Collector/Velocity(mps)", velocity);
    SmartDashboard.putNumber("Collector/TargetVelocity(mps)", limiter.lastValue());
  }

  @Override
  public void initSendable(SendableBuilder builder)
  {
    super.initSendable(builder);
    builder.addDoubleProperty("Velocity", this::getVelocity, null);
    builder.addDoubleProperty("Force", this::getForce, null);
  }
}
