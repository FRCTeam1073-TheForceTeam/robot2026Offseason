// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase 
{
  private TalonFX motor;
  private CANcoder encoder;
  private VelocityVoltage motorVelocityVoltage;
  private PositionVoltage motorPositionVoltage;
  public Debouncer climberDebouncer = new Debouncer(0.05);

  private final double motorKP = 0.15;
  private final double motorKI = 0;
  private final double motorKD = 0;
  private final double motorKV = 0.12;

  private double velocity = 0;
  private double position = 0;
  private double absolutePosition = 0;
  private double load = 0;
  private double commandedVelocity = 0;
  private boolean brakeMode = true;

  public final double minPosition = -0.31;
  public final double maxPosition = 0.4;


  /** Creates a new Climber. */
  public Climber() 
  {
    motor = new TalonFX(15);
    encoder = new CANcoder(16);

    motorVelocityVoltage = new VelocityVoltage(0).withSlot(0);
    motorPositionVoltage = new PositionVoltage(0).withSlot(0);

    configureHardware();
  }

  @Override
  public void periodic() 
  {
    velocity = motor.getVelocity().getValueAsDouble(); 
    position = motor.getPosition().getValueAsDouble();
    load = motor.getTorqueCurrent().getValueAsDouble();
    absolutePosition = encoder.getAbsolutePosition().getValueAsDouble();
    if(absolutePosition > maxPosition && commandedVelocity > 0){
      commandedVelocity = 0;
    }
    if(absolutePosition < minPosition && commandedVelocity < 0){
      commandedVelocity = 0;
    }
    motor.setControl(motorVelocityVoltage.withVelocity(commandedVelocity));

    SmartDashboard.putNumber("Climber/Commanded velocity",commandedVelocity);
    SmartDashboard.putNumber("Climber/Velocity",velocity);
    SmartDashboard.putNumber("Climber/Absolute position", absolutePosition);
  }

  public void setCommandedVelocity(double velocity)
  {
    commandedVelocity = velocity;
  }

  public double getVelocity()
  {
    return velocity;
  }

  public double getMotorPosition()
  {
    return position;
  }

  public double getLoad()
  {
    return load;
  }

  public double getEncoderPosition()
  {
    return absolutePosition;
  }

  public boolean getIsAtZero(){
    return Math.abs(getEncoderPosition()) < 0.05;
  }

  public boolean getIsDisengaged(){
    return Math.abs(getEncoderPosition() - maxPosition) < 0.04;
  }

  public boolean getIsEngaged(){
    return Math.abs(getEncoderPosition() - minPosition) < 0.01;
  }

  public double getMinPosition(){
    return minPosition;
  }

  public double getMaxPosition(){
    return maxPosition;
  }

  public void setZero()
  {
    position = 0;
    motor.setPosition(0);
  }

  public void setBrakeMode(boolean mode)
  {
    if (mode)
    {
      motor.setNeutralMode(NeutralModeValue.Brake);
    }
    else
    {
      motor.setNeutralMode(NeutralModeValue.Coast);
    }
    brakeMode = mode;
  }

  public void configureHardware()
  {
    var motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motor.getConfigurator().apply(motorConfig);

    var motorClosedLoopConfig = new SlotConfigs();
    motorClosedLoopConfig.withKP(motorKP);
    motorClosedLoopConfig.withKI(motorKI);
    motorClosedLoopConfig.withKD(motorKD);
    motorClosedLoopConfig.withKV(motorKV);

    motor.getConfigurator().apply(motorClosedLoopConfig, 0.5);

    motor.setNeutralMode(NeutralModeValue.Coast);

    CurrentLimitsConfigs motorCurrentLimitsConfigs = new CurrentLimitsConfigs();
    motorCurrentLimitsConfigs.withSupplyCurrentLimitEnable(true)
                            .withSupplyCurrentLimit(15)
                            .withSupplyCurrentLowerTime(0.25);

    motor.getConfigurator().apply(motorCurrentLimitsConfigs);
   
    motor.setPosition(0);

    System.out.println("Climber: Configured");
  }
}
