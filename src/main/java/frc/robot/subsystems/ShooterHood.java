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
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.DashboardNames;

public class ShooterHood extends SubsystemBase
{
  public static final int HoodMotorId = 24;

  public static final double ampsPerNewtonMeter = 10.0;
  public static final double hoodToMotorGearRatio = (52.0 / 12.0) * (33.0 / 15.0) * (160.0 / 10.0);
  public static final double currentLimit = 25.0;

  public static final double maxPositionRadians = Math.toRadians(69.0);
  public static final double minPositionRadians = Math.toRadians(42.0);

  private enum Mode { NONE, VELOCITY, POSITION }

  private final TalonFX motor;
  private final StatusSignal<Angle> positionSig;
  private final StatusSignal<Current> currentSig;
  // Slot assignment is intentionally swapped relative to Turret/Intake: velocity uses Slot0, position uses Slot1.
  private final PositionVoltage commandPositionVoltage = new PositionVoltage(0).withSlot(1);
  private final VelocityVoltage commandVelocityVoltage = new VelocityVoltage(0).withSlot(0);
  private final SlewRateLimiter limiter = new SlewRateLimiter(3.0);

  private Mode mode = Mode.NONE;
  private double targetVelocity = 0.0;
  private double targetPosition = 0.0;

  private boolean hasZero = false;
  private double position = 0.0;
  private double torque = 0.0;

  public ShooterHood()
  {
    setName("ShooterHood");

    motor = new TalonFX(HoodMotorId, new CANBus("rio"));
    positionSig = motor.getPosition();
    currentSig = motor.getTorqueCurrent();

    boolean hardwareConfigured = configureHardware();
    if (!hardwareConfigured) {
      System.err.println("ShooterHood: Hardware Failed To Configure!");
    }
    SmartDashboard.putBoolean(DashboardNames.HOOD_HW_CONFIGURED.getKey(), hardwareConfigured);
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

    // Slot 0 for velocity control mode:
    configs.Slot0.kV = 0.153;
    configs.Slot0.kP = 0.15;
    configs.Slot0.kI = 0.0;
    configs.Slot0.kD = 0.01;
    configs.Slot0.kA = 0.0;

    // Slot 1 for position control mode:
    configs.Slot1.kV = 0.153;
    configs.Slot1.kP = 2.5;
    configs.Slot1.kI = 0.2;
    configs.Slot1.kD = 0.0;
    configs.Slot1.kA = 0.0;
    configs.Slot1.kS = 0.02;

    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    var status = motor.getConfigurator().apply(configs, 1.0);
    if (!status.isOK()) {
      System.err.println("ShooterHood: Control Failed To Configure!");
      return false;
    }

    motor.setPosition((69.2 / 360.0) * hoodToMotorGearRatio);

    status = motor.setNeutralMode(NeutralModeValue.Coast, 1.0);
    if (!status.isOK()) {
      System.err.println("ShooterHood: Neutral mode brake Failed To Configure!");
      return false;
    }

    return true;
  }

  // Command the hood to move at the given angular velocity, radians/second.
  public void setVelocity(double radiansPerSecond)
  {
    mode = Mode.VELOCITY;
    targetVelocity = radiansPerSecond;
  }

  // Command the hood to the given position, radians. Clamped to [minPositionRadians, maxPositionRadians].
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
    motor.setPosition((69.2 / 360.0) * hoodToMotorGearRatio);
    hasZero = true;
  }

  public double getPositionRadians()
  {
    return position;
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

    double targetAngle = 0.0;

    torque = currentSig.getValueAsDouble() / ampsPerNewtonMeter;
    position = positionSig.getValueAsDouble() / hoodToMotorGearRatio * 2.0 * Math.PI;

    if (mode == Mode.VELOCITY) {
      double motorAngularVelocity = targetVelocity * hoodToMotorGearRatio / (2.0 * Math.PI);
      motor.setControl(commandVelocityVoltage.withVelocity(motorAngularVelocity));
      limiter.reset(position); // Keep the limiter in sync in other control mode.
    } else if (mode == Mode.POSITION) {
      double clampedCommand = MathUtil.clamp(targetPosition, minPositionRadians, maxPositionRadians);
      targetAngle = limiter.calculate(clampedCommand);

      double motorAngle = targetAngle * hoodToMotorGearRatio / (2.0 * Math.PI);

      motor.setControl(commandPositionVoltage.withPosition(motorAngle));
    } else {
      motor.setControl(new NeutralOut());
      limiter.reset(position); // Keep the limiter in sync in other control mode.
    }

    SmartDashboard.putNumber(DashboardNames.HOOD_ANGLE.getKey(), position);
    SmartDashboard.putNumber(DashboardNames.HOOD_TORQUE.getKey(), torque);
    SmartDashboard.putNumber(DashboardNames.HOOD_TARGET.getKey(), targetAngle);
  }
}
