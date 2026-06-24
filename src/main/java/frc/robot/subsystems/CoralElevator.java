// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.concurrent.BlockingDeque;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*TODO: make position controlled
 * L2 Position: 17.3
 * L3 Position: 28.2
 * TODO: increase maxPosition - test somewhere with high ceilings
 */

public class CoralElevator extends SubsystemBase {
  private final String kCANbus = "rio";
  private final double velocityKP = 0.2;
  private final double velocityKD = 0.0;
  private final double velocityKI = 0.01;
  private final double velocityKV = 0.12; // Kraken kV value.
  private final double velocityKA = 0.01;

  private final double positionKP = 0.2;
  private final double positionKG = 0.15; //gravity
  private final double positionKD = 0.03;
  private final double positionKI = 0.18;
  private final double positionKV = 0.12; 
  private final double positionKA = 0.0;
  private final double positionKS = 0.15;

  private final double maxLoad = 60.0; // TODO: Tune max load.
  private final double maxPosition = 1.84;
  public final double minPosition = 0.3467; //WINDHAM


  private double position;
  private double velocity;
  private double backLoad;
  private double frontLoad;
  private double load;
  private double commandedVelocity;
  private double commandedPosition;
  private boolean brakemode;
  private boolean isAtZero;
  private boolean velocityMode = true;

  private TalonFX backElevatorMotor, frontElevatorMotor;
  private VelocityVoltage frontElevatorMotorVelocityVoltage;
  private MotionMagicVoltage frontPositionController;
  private DigitalInput zeroSensor;
  public Debouncer zeroDebouncer = new Debouncer(0.05);


  public CoralElevator() {
    frontElevatorMotor = new TalonFX(20, kCANbus);
    backElevatorMotor = new TalonFX(19, kCANbus);
    brakemode = true;

    commandedVelocity = 0.0;

    frontElevatorMotorVelocityVoltage = new VelocityVoltage(0).withSlot(0);
    frontPositionController = new MotionMagicVoltage(0).withSlot(1);

    zeroSensor = new DigitalInput(4);

    configureHardware();
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    // TODO: Need to use scale factors from ratio, etc. units need to be meters.
    velocity = frontElevatorMotor.getVelocity().getValueAsDouble();
    position = frontElevatorMotor.getPosition().getValueAsDouble();

    frontLoad = frontElevatorMotor.getTorqueCurrent().getValueAsDouble();
    backLoad = backElevatorMotor.getTorqueCurrent().getValueAsDouble();
    load = Math.max(backLoad, frontLoad);

    boolean hitHardStop = (commandedVelocity < 0.0) && (Math.abs(load) > maxLoad); // MOving down, peak load => reset.
    isAtZero = zeroDebouncer.calculate(!zeroSensor.get() | hitHardStop); // Compute debounced logical or.

    if (isAtZero){
      setZero();
      if (commandedVelocity < 0.0) commandedVelocity = 0.0; // Velocity hard-limit at bottom of travel can only go up from here.
      if(commandedPosition <= minPosition){
        commandedPosition = position;
      }
    }

    if (position > getEncoderUnits(maxPosition) && commandedVelocity > 0.0) commandedVelocity = 0.0; // Don't go past maximum height.

    if(velocityMode) {
      frontElevatorMotor.setControl(frontElevatorMotorVelocityVoltage.withVelocity(commandedVelocity).withSlot(0));
      commandedPosition = position;
    }
    else {
      frontElevatorMotor.setControl(frontPositionController.withPosition(commandedPosition).withSlot(1));
    }

    SmartDashboard.putBoolean("Coral Elevator/At zero", isAtZero);
    SmartDashboard.putBoolean("Coral Elevator/Brake mode", brakemode);
    SmartDashboard.putNumber("Coral Elevator/Position (raw encoder)", position);
    SmartDashboard.putNumber("Coral Elevator/Position (meters)", getMeters());
    SmartDashboard.putNumber("Coral Elevator/Velocity", velocity);
    SmartDashboard.putNumber("Coral Elevator/Commanded velocity", commandedVelocity);
    SmartDashboard.putBoolean("Coral Elevator/Hit hardstop", hitHardStop);
    SmartDashboard.putNumber("Coral Elevator/Load", load);
  }

  public double getPosition(){// where motor is
    return position;
  }

  public double getEncoderUnits(double meters) {
    return (28.5353 * meters) - 9.832;
  }

  public void setZero(){
    position = 0.0;
    frontElevatorMotor.setPosition(0);
    backElevatorMotor.setPosition(0);
  }

  public void setVelocity(double velocity) {
    // TODO: Convert command internall from meters/second to internal commandedVelocity value using ratio, etc.
    velocityMode = true;
    commandedVelocity = velocity;
  }

  public void setPosition(double position) {
    //this method is for mode control to hold motor positions
    velocityMode = false;
    commandedPosition = getEncoderUnits(position); //position in meters commanded position in encoder units
  }

  public boolean isCoralElevatorAtBottom(){
    return isAtZero;
  }


  public double getMaxLoad(){
    return maxLoad;
  }

  public boolean getBrakeMode(){
    return brakemode;
  }

  public double getMeters() {
    return ((position + 9.832) / 28.353); // 5:1 gear rotatio turns the pulley 0.17635 meters
  }

  public double getMeters(double encoderValue) {
    return ((encoderValue + 9.832) / 28.353);
  }

  public void setBrakeMode(boolean mode){
    if (mode){
      backElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
      frontElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    }
    else{
      backElevatorMotor.setNeutralMode(NeutralModeValue.Coast);
      frontElevatorMotor.setNeutralMode(NeutralModeValue.Coast);
    }
    brakemode = mode;
  }

  public void configureHardware() {

    var frontElevatorMotorConfig = new TalonFXConfiguration();//TODO check configs with robots
    frontElevatorMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    var mmConfigs = frontElevatorMotorConfig.MotionMagic;
    mmConfigs.MotionMagicCruiseVelocity = 83;
    mmConfigs.MotionMagicAcceleration = 150;
    mmConfigs.MotionMagicJerk = 900;

    var frontElevatorMotorClosedLoop0Config =  frontElevatorMotorConfig.Slot0;
    var frontElevatorMotorClosedLoop1Config = frontElevatorMotorConfig.Slot1;

    frontElevatorMotorClosedLoop0Config.withKP(velocityKP);
    frontElevatorMotorClosedLoop0Config.withKI(velocityKI);
    frontElevatorMotorClosedLoop0Config.withKD(velocityKD);
    frontElevatorMotorClosedLoop0Config.withKV(velocityKV);
    frontElevatorMotorClosedLoop0Config.withKA(velocityKA);

    frontElevatorMotorClosedLoop1Config.withKP(positionKP);
    frontElevatorMotorClosedLoop1Config.withKI(positionKI);
    frontElevatorMotorClosedLoop1Config.withKD(positionKD);
    frontElevatorMotorClosedLoop1Config.withKV(positionKV);
    frontElevatorMotorClosedLoop1Config.withKA(positionKA);
    frontElevatorMotorClosedLoop1Config.withKS(positionKS);
    frontElevatorMotorClosedLoop1Config.withKG(positionKG);

    frontElevatorMotorConfig.CurrentLimits.withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(30)
        .withSupplyCurrentLowerTime(0.25);
    frontElevatorMotor.getConfigurator().apply(frontElevatorMotorConfig, 0.5);


    var backElevatorMotorConfig = new TalonFXConfiguration();
    backElevatorMotorConfig.CurrentLimits.withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(30)
        .withSupplyCurrentLowerTime(0.25);
    backElevatorMotor.getConfigurator().apply(backElevatorMotorConfig, 0.5);
     // Same config as other motor to start.

    //TODO consider changing brakemode (also test ungeared setup before gearing)
    frontElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    backElevatorMotor.setNeutralMode(NeutralModeValue.Brake);

    // CurrentLimitsConfigs frontElevatorCurrentLimitsConfigs = new CurrentLimitsConfigs();
    // frontElevatorCurrentLimitsConfigs.withSupplyCurrentLimitEnable(true)
    //                         .withSupplyCurrentLimit(25)
    //                         .withSupplyCurrentLowerTime(0.25);

    // frontElevatorMotor.getConfigurator().apply(frontElevatorCurrentLimitsConfigs);
    // backElevatorMotor.getConfigurator().apply(frontElevatorCurrentLimitsConfigs);

    backElevatorMotor.setControl(new Follower(frontElevatorMotor.getDeviceID(), true));

    backElevatorMotor.setPosition(0);
    frontElevatorMotor.setPosition(0);

    System.out.println("Coral Elevator configured");
  }
}