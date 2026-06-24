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


// For the LaserCAN Sensor:
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;
import edu.wpi.first.epilogue.logging.LazyBackend;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/** Add your docs here. */
public class CoralEndeffector extends SubsystemBase {
    private final String kCANbus = "rio";
    private final double leftKP = 0.15;
    private final double leftKD = 0.005;
    private final double leftKI = 0.0;
    private final double leftKS = 0.1;
    private final double leftKV = 0.12; // Kraken.

    private final double minCoralDistance = 0.03;
    private final double maxFedDistance = 0.1;

    private double velocity;
    private double position;
    private double load;
    private double commandedVelocity;
    private double commandedAlgaeVel;
    private double commandedPosition;
    private double coralDistance;
    private static boolean hasCoral = false;
    private boolean coralFed = false;
    private boolean lastCoralFed = false;
    private boolean velocityMode = true;

    private double funnelDistance;
    private boolean ConfigurationFailedException = false;

    private TalonFX motor;
    private VelocityVoltage motorVelocityVoltage;
    private PositionVoltage motorPositionVoltage;
   
    // LaserCAN Sensor:
    private LaserCan laserCANCoral;
    private LaserCan laserCANReef;
    private LinearFilter filter;

    public CoralEndeffector() {
        hasCoral = false;
        motor = new TalonFX(21, kCANbus);
        filter = LinearFilter.singlePoleIIR(0.5, 0.02);
        
        motorVelocityVoltage = new VelocityVoltage(0).withSlot(0);
        motorPositionVoltage = new PositionVoltage(0).withSlot(0);

        // Sensor setup:
        laserCANCoral = new LaserCan(22);
        laserCANReef = new LaserCan(24);

        configureHardware();
    }

    @Override
    public void periodic() {

        // TODO: Scale factors.
        velocity = motor.getVelocity().getValueAsDouble();
        position = motor.getPosition().getValueAsDouble();

        load = filter.calculate(Math.abs(motor.getTorqueCurrent().getValueAsDouble()));
        commandedPosition = commandedPosition + (commandedAlgaeVel * 0.02);
        
        // Read the coral sensor.
        LaserCan.Measurement coral_measurement = laserCANCoral.getMeasurement();
        if (coral_measurement != null && coral_measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            coralDistance = coral_measurement.distance_mm * 0.001; // mm's
            if (coralDistance < minCoralDistance) hasCoral = true;
            else hasCoral = false;
        } else {
            coralDistance = 999.0;
            hasCoral = false;
        }

        // Read the funnel sensor.
        // TODO: set to short range
        LaserCan.Measurement funnel_measurement = laserCANReef.getMeasurement();
        if (funnel_measurement != null && funnel_measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            funnelDistance = funnel_measurement.distance_mm * 0.001; // mm's
            if (funnelDistance > maxFedDistance && lastCoralFed){
                coralFed = true;
            }
            lastCoralFed = funnelDistance <= maxFedDistance;
        }
        // Send motor command:
        if(velocityMode){
            motor.setControl(motorVelocityVoltage.withVelocity(commandedVelocity));
        }
        else{
            motor.setControl(motorPositionVoltage.withPosition(commandedPosition));
        }

        SmartDashboard.putNumber("Coral End Effector/Coral Distance", coralDistance);
        SmartDashboard.putBoolean("Coral End Effector/Has Coral", hasCoral);
        SmartDashboard.putNumber("Coral End Effector/ velocity", velocity);
        SmartDashboard.putNumber("Coral End Effector/command", commandedVelocity);
        SmartDashboard.putNumber("Coral End Effector/load", load);
        SmartDashboard.putNumber("Coral End Effector/Reef Distance", funnelDistance);
        SmartDashboard.putBoolean("Coral End Effector/Coral Fed", coralFed);
        SmartDashboard.putBoolean("Coral End Effector/Last Coral Fed", lastCoralFed);
    
    }
    
    // TODO: Update when openMV is implemented
    public boolean getHasReef(){
        return true;
    }

    public void setCoralFed(boolean fed){
        coralFed = fed;
    }

    public double getPosition(){
        return position;
    }

    public void setVelocity(double velocity){
        // TODO: Scale factors.
        velocityMode = true;
        this.commandedVelocity = -velocity;
    }

    public void setAlgaeVelocity(double vel){
        velocityMode = false;
        commandedAlgaeVel = -vel;
    }

    public double getVelocity() {
        return velocity;
    }

    public double getLoad() {
        return Math.abs(load);
    }

    public double getCoralDistance() {
        return coralDistance;
    }

    public static boolean getHasCoral() {
        return hasCoral;
    }

    public double getFunnelDistance() {
        return funnelDistance;
    }

    public boolean getCoralFed() {
        return coralFed;
    }


    public void configureHardware() {

    var motorConfig = new TalonFXConfiguration(); //TODO check configs with robots
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motor.getConfigurator().apply(motorConfig);

    var motorClosedLoopConfig = new SlotConfigs();
    motorClosedLoopConfig.withKP(leftKP);
    motorClosedLoopConfig.withKI(leftKI);
    motorClosedLoopConfig.withKD(leftKD);
    motorClosedLoopConfig.withKV(leftKV);
    motorClosedLoopConfig.withKS(leftKS);

    var error = motor.getConfigurator().apply(motorClosedLoopConfig, 0.5);

    motor.setNeutralMode(NeutralModeValue.Coast);//TODO consider changing brakemode (also test ungeared setup before gearing)

    motor.setPosition(0);

    // Laser CAN Setup:
    try {
        laserCANCoral.setRangingMode(LaserCan.RangingMode.SHORT);
        laserCANCoral.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 8, 8));
        laserCANCoral.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
      } catch (ConfigurationFailedException e) {
        System.out.println("Configuration failed for LaserCAN: " + e);
      }

    System.out.println("Coral Endeffector was configured");
  }
}
