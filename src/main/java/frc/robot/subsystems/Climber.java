// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.DashboardNames;

public class Climber extends SubsystemBase
{
  public static final int ClimberMotorId = 29;

  public static final double gearRatio = 32.0;
  public static final double turnsPerMeter = 1.0 / (0.75 * 0.0254 * Math.PI);
  public static final double ampsPerNewton = 10.0;
  public static final double currentLimit = 67.0;

  public static final double minPositionMeters = 0.0;
  public static final double maxPositionMeters = 0.07;

  private enum Mode { NONE, VELOCITY, POSITION }

  private final TalonFX motor;
  private final StatusSignal<AngularVelocity> velocitySig;
  private final StatusSignal<Current> currentSig;
  private final StatusSignal<Angle> positionSig;
  private final VelocityVoltage commandVelocityVoltage = new VelocityVoltage(0).withSlot(0);
  private final PositionVoltage commandPositionVoltage = new PositionVoltage(0).withSlot(1);
  private final SlewRateLimiter limiter = new SlewRateLimiter(10.0);
  private final SlewRateLimiter positionLimiter = new SlewRateLimiter(0.3);
  private final DigitalInput climberOnInput = new DigitalInput(0);

  private Mode mode = Mode.NONE;
  private double targetVelocity = 0.0;
  private double targetPosition = 0.0;

  private double position = 0.0;
  private double velocity = 0.0;
  private double force = 0.0;

  public Climber()
  {
    setName("Climber");

    motor = new TalonFX(ClimberMotorId, new CANBus("Canivore"));
    velocitySig = motor.getVelocity();
    currentSig = motor.getTorqueCurrent();
    positionSig = motor.getPosition();

    boolean hardwareConfigured = configureHardware();
    if (!hardwareConfigured) {
      System.err.println("Climber: Hardware Failed To Configure!");
    }
    SmartDashboard.putBoolean(DashboardNames.CLIMBER_HW_CONFIGURED.getKey(), hardwareConfigured);
  }

  private boolean configureHardware()
  {
    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.TorqueCurrent.PeakForwardTorqueCurrent = 10.0;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -10.0;

    configs.Voltage.PeakForwardVoltage = 8.0;
    configs.Voltage.PeakReverseVoltage = -8.0;

    configs.CurrentLimits.SupplyCurrentLimit = currentLimit;
    configs.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Slot 0 for the velocity control loop:
    configs.Slot0.kV = 0.12;
    configs.Slot0.kP = 0.15;
    configs.Slot0.kI = 0.0;
    configs.Slot0.kD = 0.01;
    configs.Slot0.kA = 0.0;
    configs.Slot0.kS = 0.0;

    // Slot 1 for the position control loop:
    configs.Slot1.kV = 0.12;
    configs.Slot1.kP = 0.3;
    configs.Slot1.kI = 0.03;
    configs.Slot1.kD = 0.01;
    configs.Slot1.kA = 0.0;
    configs.Slot1.kS = 0.03;

    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    var status = motor.getConfigurator().apply(configs, 1.0);
    if (!status.isOK()) {
      System.err.println("Climber: config failed to config!");
      return false;
    }

    motor.setPosition(0);

    status = motor.setNeutralMode(NeutralModeValue.Brake, 1.0);
    if (!status.isOK()) {
      System.err.println("Climber: neutral mode failed to config :(!");
      return false;
    }

    return true;
  }

  // Command the climber to move at the given velocity, meters/second.
  public void setVelocity(double metersPerSecond)
  {
    mode = Mode.VELOCITY;
    targetVelocity = metersPerSecond;
    SmartDashboard.putNumber(DashboardNames.CLIMBER_COMMANDED_VELOCITY.getKey(), metersPerSecond);
  }

  // Command the climber to the given position, meters. Clamped to [minPositionMeters, maxPositionMeters].
  public void setPosition(double meters)
  {
    mode = Mode.POSITION;
    targetPosition = meters;
    SmartDashboard.putNumber(DashboardNames.CLIMBER_COMMANDED_POSITION.getKey(), meters);
  }

  public void stop()
  {
    mode = Mode.NONE;
  }

  public void zero()
  {
    motor.setPosition(0);
  }

  public boolean isHooked()
  {
    return climberOnInput.get();
  }

  public double getClimberPosition()
  {
    return position;
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
    positionSig.refresh();

    force = currentSig.getValueAsDouble() / ampsPerNewton;
    velocity = velocitySig.getValueAsDouble() / (turnsPerMeter * gearRatio);
    position = positionSig.getValueAsDouble() / (turnsPerMeter * gearRatio);

    if (mode == Mode.VELOCITY) {
      double limitedVel = limiter.calculate(targetVelocity);
      double motorVel = limitedVel * turnsPerMeter * gearRatio;

      motor.setControl(commandVelocityVoltage.withVelocity(motorVel));
      positionLimiter.reset(position);
    } else if (mode == Mode.POSITION) {
      double limitedPos = positionLimiter.calculate(targetPosition);
      double clampedCommand = MathUtil.clamp(limitedPos, minPositionMeters, maxPositionMeters);

      double motorPosition = clampedCommand * turnsPerMeter * gearRatio;

      motor.setControl(commandPositionVoltage.withPosition(motorPosition));

      SmartDashboard.putNumber(DashboardNames.CLIMBER_TARGET_POSITION.getKey(), clampedCommand);
      SmartDashboard.putNumber(DashboardNames.CLIMBER_LAST_COMMAND.getKey(), clampedCommand);
    } else {
      motor.setControl(new NeutralOut());
      limiter.reset(0.0);
      positionLimiter.reset(position);
    }

    SmartDashboard.putNumber(DashboardNames.CLIMBER_VELOCITY.getKey(), velocity);
    SmartDashboard.putNumber(DashboardNames.CLIMBER_TARGET_VELOCITY.getKey(), limiter.lastValue());
    SmartDashboard.putNumber(DashboardNames.CLIMBER_LOAD.getKey(), Math.abs(force));
    SmartDashboard.putNumber(DashboardNames.CLIMBER_POSITION.getKey(), position);
  }
}
