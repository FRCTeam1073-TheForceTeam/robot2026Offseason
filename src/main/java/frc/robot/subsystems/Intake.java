// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.DashboardNames;

public class Intake extends SubsystemBase
{
  public static final int IntakeLeadId = 18;
  public static final int IntakeFollowId = 19;

  public static final double gearRatio = 40.0; // From new design.
  public static final double ampsPerNewtonMeter = 10.0;
  public static final double currentLimit = 45.0;

  public static final double maxPositionRadians = 0.0;
  public static final double minPositionRadians = -2.13;

  private enum Mode { NONE, VELOCITY, POSITION }

  private final TalonFX leadMotor;
  private final TalonFX followMotor;
  private final StatusSignal<Angle> positionSig;
  private final StatusSignal<AngularVelocity> velocitySig;
  private final StatusSignal<Current> currentSig;
  private final PositionVoltage commandPositionVoltage = new PositionVoltage(0).withSlot(0);
  private final VelocityVoltage commandVelocityVoltage = new VelocityVoltage(0).withSlot(1);
  private final SlewRateLimiter limiter = new SlewRateLimiter(9.0); // was 5 radians before

  private Mode mode = Mode.NONE;
  private double targetVelocity = 0.0;
  private double targetPosition = 0.0;

  private boolean hasZero = false;
  private double position = 0.0;
  private double velocity = 0.0;
  private double torque = 0.0;

  public Intake()
  {
    setName("Intake");

    leadMotor = new TalonFX(IntakeLeadId, new CANBus("rio"));
    followMotor = new TalonFX(IntakeFollowId, new CANBus("rio"));
    positionSig = leadMotor.getPosition();
    velocitySig = leadMotor.getVelocity();
    currentSig = leadMotor.getTorqueCurrent();

    boolean hardwareConfigured = configureHardware();
    if (!hardwareConfigured) {
      System.err.println("Intake: Hardware Failed To Configure!");
    }
    SmartDashboard.putBoolean(DashboardNames.INTAKE_HW_CONFIGURED.getKey(), hardwareConfigured);
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

    // Slot 0 for the position control loop:
    configs.Slot0.kV = 0.153;
    configs.Slot0.kP = 0.4;
    configs.Slot0.kI = 0.04;
    configs.Slot0.kD = 0.01;
    configs.Slot0.kA = 0.0;
    configs.Slot0.kS = 0.02;

    // Slot 1 for the velocity control loop:
    configs.Slot1.kV = 0.153;
    configs.Slot1.kP = 0.3;
    configs.Slot1.kI = 0.0;
    configs.Slot1.kD = 0.0;
    configs.Slot1.kA = 0.0;
    configs.Slot1.kS = 0.02;

    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    var status = leadMotor.getConfigurator().apply(configs, 1.0);
    if (!status.isOK()) {
      System.err.println("Intake: leader failed to config!");
      return false;
    }

    // The follow motor's actual physical inversion is handled via Follower(Opposed) in periodic(), so
    // it's configured with the same base config as the leader here (matching current C++ behavior).
    status = followMotor.getConfigurator().apply(configs, 1.0);
    if (!status.isOK()) {
      System.err.println("Intake: follower failed to config!");
      return false;
    }

    // Set our neutral mode to brake on:
    status = leadMotor.setNeutralMode(NeutralModeValue.Brake, 1.0);
    if (!status.isOK()) {
      System.err.println("Intake: neutral mode failed to config :(!");
      return false;
    }

    // Initialize at the zero position:
    leadMotor.setPosition((-122.0 / 360.0) * gearRatio);

    return true;
  }

  // Command the intake to spin at the given angular velocity, radians/second.
  public void setVelocity(double radiansPerSecond)
  {
    mode = Mode.VELOCITY;
    targetVelocity = radiansPerSecond;
  }

  // Command the intake pivot to the given position, radians. Clamped to [minPositionRadians, maxPositionRadians].
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
    leadMotor.setPosition((-122.0 / 360.0) * gearRatio);
    hasZero = true;
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
    return hasZero;
  }

  @Override
  public void periodic()
  {
    positionSig.refresh();
    currentSig.refresh();
    velocitySig.refresh();

    torque = currentSig.getValueAsDouble() / ampsPerNewtonMeter;
    position = positionSig.getValueAsDouble() * 2.0 * Math.PI / gearRatio;
    velocity = velocitySig.getValueAsDouble() * 2.0 * Math.PI / gearRatio;

    if (mode == Mode.VELOCITY) {
      double motorVelocity = targetVelocity * gearRatio / (2.0 * Math.PI);
      leadMotor.setControl(commandVelocityVoltage.withVelocity(motorVelocity));
      followMotor.setControl(new Follower(leadMotor.getDeviceID(), MotorAlignmentValue.Opposed));
      limiter.reset(position); // Keep the limiter in sync in the other control mode.
    } else if (mode == Mode.POSITION) {
      double clampedCommand = MathUtil.clamp(targetPosition, minPositionRadians, maxPositionRadians);
      // Matches C++: the limiter is evaluated against the clamped command first (its result is
      // discarded), then evaluated again against the raw command for the value actually used -
      // a stateful double-Calculate() quirk in the source, preserved here rather than "fixed".
      limiter.calculate(clampedCommand);
      double limitedIntakeTarget = limiter.calculate(targetPosition);

      double motorPosition = limitedIntakeTarget * gearRatio / (2.0 * Math.PI);

      leadMotor.setControl(commandPositionVoltage.withPosition(motorPosition));
      followMotor.setControl(new Follower(leadMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    } else {
      leadMotor.setControl(new NeutralOut());
      followMotor.setControl(new NeutralOut());
      limiter.reset(position); // Keep the limiter in sync in other control mode.
    }

    SmartDashboard.putNumber(DashboardNames.INTAKE_POSITION.getKey(), position);
    SmartDashboard.putNumber(DashboardNames.INTAKE_TARGET_POSITION.getKey(), limiter.lastValue());
    SmartDashboard.putNumber(DashboardNames.INTAKE_TORQUE.getKey(), torque);
  }
}
