// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaePivot extends SubsystemBase {

  private final double rotateMinPos = 0;
  private final double rotateMaxPos = 17.13;

  private final double rotateVelKP = 0.2;
  private final double rotateVelKI = 0.01;
  private final double rotateVelKD = 0.0;
  private final double rotateVelKV = 0.12;
  private final double rotateVelKA = 0.01;

  private final double rotatePosKP = 0.2;
  private final double rotatePosKI = 0.05;
  private final double rotatePosKD = 0.0;
  private final double rotatePosKV = 0.12;
  private final double rotatePosKA = 0.01;

  private double rotateVel;
  private double commandedRotateVel;
  private double commandedRotatePos;
  private double rotatePos;
  private double rotateLoad;

  private boolean rotateBrakeMode = true;
  private boolean isUp = true;
  private boolean velocityMode = true;

  private TalonFX rotateMotor;
  private VelocityVoltage rotateVelocityVoltage;
  private MotionMagicVoltage rotatePositionController;
  private LinearFilter filter;

  public AlgaePivot() {
    filter = LinearFilter.singlePoleIIR(0.5, 0.02);
    rotateMotor = new TalonFX(26);
    rotateBrakeMode = true;
    velocityMode = true;
    isUp = true;

    commandedRotateVel = 0.0;

    rotateVelocityVoltage = new VelocityVoltage(0).withSlot(0);
    rotatePositionController = new MotionMagicVoltage(0).withSlot(1);

    configureHardware();
  }

  @Override
  public void periodic() {

    rotateVel = rotateMotor.getVelocity().getValueAsDouble(); 
    rotatePos = rotateMotor.getPosition().getValueAsDouble();
    rotateLoad = filter.calculate(Math.abs(rotateMotor.getTorqueCurrent().getValueAsDouble()));
    
    //commandedRotatePos = commandedRotatePos + (commandedRotateVel * 0.02); //calculating collect position based on velocity and time

    if (rotatePos >= rotateMaxPos){
      commandedRotateVel = Math.min(commandedRotateVel, 0); 
    }
    // if (rotatePos <= rotateMinPos){
    //   commandedRotateVel = Math.max(commandedRotateVel, 0);
    // }

    if(velocityMode) {
      rotateMotor.setControl(rotateVelocityVoltage.withVelocity(commandedRotateVel).withSlot(0));
    }
    else {
      rotateMotor.setControl(rotatePositionController.withPosition(commandedRotatePos).withSlot(1));
    }

    SmartDashboard.putNumber("Algae Pivot/Rotate Velocity", rotateVel);
    SmartDashboard.putNumber("Algae Pivot/Rotate Commanded Velocity", commandedRotateVel);
    SmartDashboard.putNumber("Algae Pivot/Rotate Commanded Position", commandedRotatePos);
    SmartDashboard.putNumber("Algae Pivot/Rotate Position", rotatePos);
    SmartDashboard.putNumber("Algae Pivot/Rotate Motor Load", rotateLoad);
    SmartDashboard.putBoolean("Algae Pivot/Rotate Break Mode", !rotateBrakeMode);
    SmartDashboard.putBoolean("Algae Pivot/Is Up", isUp);
  }

  public void setRotatorVel(double newVel){
    velocityMode = true;
    commandedRotateVel = newVel;
  }

  public void setRotatorPos(double position) {
    velocityMode = false;
    commandedRotatePos = position;
  }

  public void zeroRotator(){
    rotatePos = 0.0;
    rotateMotor.setPosition(0);
  }

  public void setRotateBrakeMode(boolean mode){
    if(mode){
      rotateMotor.setNeutralMode(NeutralModeValue.Brake);
    }
    else{
      rotateMotor.setNeutralMode(NeutralModeValue.Coast);
    }
    rotateBrakeMode = mode;
  }

  public double getRotatorVelocity(){
    return rotateVel;
  }

  public double getRotatorPosition(){
    return rotatePos;
  }

  public double getRotatorLoad(){
    return Math.abs(rotateLoad);
  }

  public boolean getIsAtZero(){
    return rotatePos <= .01;
  }

  public boolean getIsUp(){
    return isUp;
  }

  public boolean getRotatorBrakeMode(){
    return rotateBrakeMode;
  }

  public void configureHardware()
  {
    var rotateMotorConfig = new TalonFXConfiguration();
    var collectMotorConfig = new TalonFXConfiguration();
    rotateMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    collectMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    var rotatemmConfigs = rotateMotorConfig.MotionMagic;
    rotatemmConfigs.MotionMagicCruiseVelocity = 75; //TODO: tune numbers
    rotatemmConfigs.MotionMagicAcceleration = 150;
    rotatemmConfigs.MotionMagicJerk = 800;  
    
    var collectMotorClosedLoop0Config = collectMotorConfig.Slot0;
    var rotateMotorClosedLoop0Config = rotateMotorConfig.Slot0;
    var rotateMotorClosedLoop1Config = rotateMotorConfig.Slot1;

    rotateMotorClosedLoop0Config.withKP(rotateVelKP);
    rotateMotorClosedLoop0Config.withKI(rotateVelKI);
    rotateMotorClosedLoop0Config.withKD(rotateVelKD);
    rotateMotorClosedLoop0Config.withKV(rotateVelKV);
    rotateMotorClosedLoop0Config.withKA(rotateVelKA);

    rotateMotorClosedLoop1Config.withKP(rotatePosKP);
    rotateMotorClosedLoop1Config.withKI(rotatePosKI);
    rotateMotorClosedLoop1Config.withKD(rotatePosKD);
    rotateMotorClosedLoop1Config.withKV(rotatePosKV);
    rotateMotorClosedLoop1Config.withKA(rotatePosKA);

    rotateMotorConfig.CurrentLimits.withSupplyCurrentLimitEnable(true)
    .withSupplyCurrentLimit(15)
    .withSupplyCurrentLowerTime(0.25);

    collectMotorConfig.CurrentLimits.withSupplyCurrentLimitEnable(true)
    .withSupplyCurrentLimit(15)
    .withSupplyCurrentLowerTime(0.25);

    rotateMotor.getConfigurator().apply(rotateMotorConfig, 0.5);

    rotateMotor.setNeutralMode(NeutralModeValue.Brake);
   
    rotateMotor.setPosition(0);

    System.out.println("Algae Rotator: Configured");
  }
}
