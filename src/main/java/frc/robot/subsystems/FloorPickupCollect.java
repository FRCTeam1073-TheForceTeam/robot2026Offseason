  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FloorPickupCollect extends SubsystemBase {
  /** Creates a new FloorPickupCollect. */
  private String kCANbus = "rio";
  private double rollerkP = 0.2;
  private double rollerkI = 0.0;
  private double rollerkD = 0.0;
  private double rollerkV = 0.0;
  private double rollerkA = 0.0;

  private double velocity = 0.0;
  private double commandedVelocity = 0.0;
  private double load;
  private double position;
  private LinearFilter filter;

  private TalonFX rollerMotor;
  private VelocityVoltage velocityVoltage;
    
  public FloorPickupCollect() {
    rollerMotor = new TalonFX(27, kCANbus);
    filter = LinearFilter.singlePoleIIR(0.5, 0.02);

    commandedVelocity = 0.0;
    velocityVoltage = new VelocityVoltage(0);

    configureHardware();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    velocity = rollerMotor.getVelocity().getValueAsDouble();
    load = filter.calculate(Math.abs(rollerMotor.getTorqueCurrent().getValueAsDouble()));
    position = rollerMotor.getPosition().getValueAsDouble();

    rollerMotor.setControl(velocityVoltage.withVelocity(commandedVelocity));

    SmartDashboard.putNumber("Floor Collect/Velocity", velocity);
    SmartDashboard.putNumber("Floor Collect/Commanded Velocity", commandedVelocity);
    SmartDashboard.putNumber("Floor Collect/Load", load);
  }

  public double getVelocity(){
    return velocity;
  }

  public void setVelocity(double vel){
    commandedVelocity = vel;
  }

  public double getLoad(){
    return load;
  }

  public double getPosition(){
    return position;
  }

  public void configureHardware() {

        var rollerMotorConfig = new TalonFXConfiguration(); //TODO check configs with robots
        rollerMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        var rollerMotorClosedLoopConfig = rollerMotorConfig.Slot0;
        rollerMotorClosedLoopConfig.withKP(rollerkP);
        rollerMotorClosedLoopConfig.withKI(rollerkI);
        rollerMotorClosedLoopConfig.withKD(rollerkD);
        rollerMotorClosedLoopConfig.withKV(rollerkV);
        rollerMotorClosedLoopConfig.withKA(rollerkA);
        //rollerMotorClosedLoopConfig.withKS(rollerkS);

        var rollerError = rollerMotor.getConfigurator().apply(rollerMotorClosedLoopConfig, 0.5);
        
        rollerMotor.getConfigurator().apply(rollerMotorConfig, 0.5);

        rollerMotor.setNeutralMode(NeutralModeValue.Brake);//TODO consider changing brakemode (also test ungeared setup before gearing)

        rollerMotor.setPosition(0);

        System.out.println("Floor Pickup was configured");
    }
}
