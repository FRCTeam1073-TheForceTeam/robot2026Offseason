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
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase
{
  public static final int RotaterMotorId = 25;
  public static final int EncoderMotorId = 26; // Currently unused: no separate absolute encoder wired in yet.

  public static final double turretToMotorTurns = (50.0 / 14.0) * (82.0 / 14.0);
  public static final double ampsPerNewtonMeter = 10.0;
  public static final double currentLimit = 40.0;
  public static final double minPositionRadians = Math.toRadians(-169.0);
  public static final double maxPositionRadians = Math.toRadians(185.0);

  private enum Mode { NONE, VELOCITY, POSITION }

  private final TalonFX motor;
  private final StatusSignal<Angle> positionSig;
  private final StatusSignal<AngularVelocity> velocitySig;
  private final StatusSignal<Current> currentSig;
  private final PositionVoltage commandPositionVoltage = new PositionVoltage(0).withSlot(0);
  private final VelocityVoltage commandVelocityVoltage = new VelocityVoltage(0).withSlot(1);
  private final SlewRateLimiter limiter = new SlewRateLimiter(8.0);

  private Mode mode = Mode.NONE;
  private double targetVelocity = 0.0;
  private double targetPosition = 0.0;

  private boolean haveZero = false;
  private double position = 0.0;
  private double velocity = 0.0;
  private double torque = 0.0;
  private boolean locked = false;

  public Turret()
  {
    setName("Turret");

    motor = new TalonFX(RotaterMotorId, new CANBus("rio"));
    positionSig = motor.getPosition();
    velocitySig = motor.getVelocity();
    currentSig = motor.getTorqueCurrent();

    boolean hardwareConfigured = configureHardware();
    if (!hardwareConfigured) {
      System.err.println("Turret: Hardware Failed To Configure!");
    }
    SmartDashboard.putBoolean("Turret/Turret - hardware_configured", hardwareConfigured);
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

    // Slot 0 for position control mode:
    configs.Slot0.kV = 0.153;
    configs.Slot0.kP = 6;
    configs.Slot0.kI = 1;
    configs.Slot0.kD = 0.2;
    configs.Slot0.kA = 0.0;
    configs.Slot0.kS = 0.05;

    // Slot 1 is velocity:
    configs.Slot1.kV = 0.153;
    configs.Slot1.kP = 0.1;
    configs.Slot1.kI = 0.0;
    configs.Slot1.kD = 0.0;
    configs.Slot1.kA = 0.0;

    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    var status = motor.getConfigurator().apply(configs, 1.0);
    if (!status.isOK()) {
      System.err.println("Turret: Control Failed To Configure!");
    }

    status = motor.setNeutralMode(NeutralModeValue.Brake, 1.0);
    if (!status.isOK()) {
      System.err.println("Turret: Neutral mode brake Failed To Configure!");
      return false;
    }

    // Optionally start out at max position after initialization, matching the physical hard-stop zero.
    motor.setPosition(maxPositionRadians / (2.0 * Math.PI) * turretToMotorTurns);

    return true;
  }

  // Command the turret to rotate at the given angular velocity, radians/second.
  public void setVelocity(double radiansPerSecond)
  {
    mode = Mode.VELOCITY;
    targetVelocity = radiansPerSecond;
  }

  // Command the turret to rotate to the given position, radians. Clamped to [minPositionRadians, maxPositionRadians].
  public void setPosition(double radians)
  {
    mode = Mode.POSITION;
    targetPosition = radians;
  }

  public void stop()
  {
    mode = Mode.NONE;
  }

  public void zero()
  {
    motor.setPosition(maxPositionRadians / (2.0 * Math.PI) * turretToMotorTurns);
    haveZero = true;
  }

  public double getTargetPositionRadians()
  {
    return targetPosition;
  }

  public double getPositionRadians()
  {
    return position;
  }

  public double getVelocityRadPerSec()
  {
    return velocity;
  }

  public double getTorqueNm()
  {
    return torque;
  }

  public boolean hasZero()
  {
    return haveZero;
  }

  public boolean isLocked()
  {
    return locked;
  }

  @Override
  public void periodic()
  {
    positionSig.refresh();
    velocitySig.refresh();
    currentSig.refresh();

    torque = currentSig.getValueAsDouble() / ampsPerNewtonMeter;
    position = positionSig.getValueAsDouble() * 2.0 * Math.PI / turretToMotorTurns;
    velocity = velocitySig.getValueAsDouble() * 2.0 * Math.PI / turretToMotorTurns;

    if (mode == Mode.VELOCITY) {
      SmartDashboard.putNumber("Turret/Target Velocity", velocity);
      double motorVelocity = targetVelocity * turretToMotorTurns / (2.0 * Math.PI);
      limiter.reset(position); // Keep the limiter in sync in other control mode.
      motor.setControl(commandVelocityVoltage.withVelocity(motorVelocity));
    } else if (mode == Mode.POSITION) {
      double clampedCommand = MathUtil.clamp(targetPosition, minPositionRadians, maxPositionRadians);
      double turretAngle = limiter.calculate(clampedCommand);
      double motorAngle = turretAngle * turretToMotorTurns / (2.0 * Math.PI);
      motor.setControl(commandPositionVoltage.withPosition(motorAngle));
    } else {
      motor.setControl(new NeutralOut());
      limiter.reset(position); // Keep the limiter in sync in other control mode.
    }

    locked = Math.abs(targetPosition - position) < Math.toRadians(2.0);

    SmartDashboard.putNumber("Turret/Position rad", position);
    SmartDashboard.putNumber("Turret/Position deg", Math.toDegrees(position));
    SmartDashboard.putNumber("Turret/Velocity (Rad_s))", velocity);
    SmartDashboard.putNumber("Turret/Target", targetPosition);
    SmartDashboard.putNumber("Turret/Torque", torque);
    SmartDashboard.putBoolean("Turret/HaveZero", haveZero);
    SmartDashboard.putBoolean("Turret/LinedUp", locked);
  }

  @Override
  public void initSendable(SendableBuilder builder)
  {
    super.initSendable(builder);
    builder.addDoubleProperty("Position", this::getPositionRadians, null);
    builder.addDoubleProperty("Velocity", this::getVelocityRadPerSec, null);
    builder.addDoubleProperty("Torque", this::getTorqueNm, null);
    builder.addBooleanProperty("HaveZero", this::hasZero, null);
    builder.addBooleanProperty("Locked", this::isLocked, null);
  }
}
