// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Swerve module class one for each swerve module. 
 * 
 * ! ! ! ! ! NOTE ! ! ! ! ! ! !
 *  
 * THIS CODE ONLY WORKS IF YOU SET THE CANCODERS MAGNETIC OFFSET TO BOOT TO ABSOLUTE POSITION
 * 
 * OTHERWISE THE WHEELS WILL NOT INITIALIZE IN THE CORRECT POSITIONS
 * 
 * ! ! ! ! ! NOTE ! ! ! ! ! ! !
*/
public class SwerveModule extends SubsystemBase implements Sendable
{
    private SwerveModuleConfig cfg;
    private SwerveModuleIDConfig idcfg;
    private TalonFX steerMotor, driveMotor;
    private CANcoder steerEncoder;
    public Translation2d position;
    public VelocityVoltage driveVelocityVoltage;
    public PositionVoltage steerPositionVoltage;
    private double targetSteerRotations = 0.0;
    private double targetDriveVelocity = 0.0;
    private double targetDriveVelocityRotations = 0.0;
    private double steerVelocity;
    private final String kCANbus = "Canivore";


    /** Constructs a swerve module class. Initializes drive and steer motors
     * 
     * @param cfg swerve module configuration values for this module
     * @param ids Can Ids for this module
     */
    public SwerveModule(SwerveModuleConfig cfg, SwerveModuleIDConfig ids)
    {
        this.position = cfg.position;
        this.cfg = cfg;
        this.idcfg = ids;

        setName(String.format("SwerveModule[%d]", cfg.moduleNumber));

        steerMotor = new TalonFX(ids.steerMotorID, kCANbus);
        driveMotor = new TalonFX(ids.driveMotorID, kCANbus);
        steerEncoder = new CANcoder(ids.steerEncoderID, kCANbus);
    
        driveVelocityVoltage = new VelocityVoltage(0).withSlot(0);
        steerPositionVoltage = new PositionVoltage(0).withSlot(0);
        configureHardware();
    }

    // Sample a SwerveModulePosition object from the state of this module.
    public void samplePosition(SwerveModulePosition position)
    {
        position.angle = Rotation2d.fromRotations(getSteerRotations());
        position.distanceMeters = getDrivePosition();
    }

    // Return steering sensor angle in rotations. 0 = dead ahead on robot.
    public double getSteerRotations()
    {
        steerEncoder.getAbsolutePosition().refresh();
        return ((steerEncoder.getAbsolutePosition().getValueAsDouble()));
    }

    // Return drive position in meters.
    public double getDrivePosition()
    {
        return driveMotor.getRotorPosition().getValueAsDouble() / cfg.rotationsPerMeter;
    }

    // Return drive velocity in meters/second.
    public double getDriveVelocity()
    { 
        return driveMotor.getRotorVelocity().getValueAsDouble() / (cfg.rotationsPerMeter);
    }
    
    public double getTargetSteerRotations() {
        return targetSteerRotations;
    }

    public double getTargetDriveVelocity()  {
        return targetDriveVelocity;
    }

    //debug only
    public double getTargetDriveVelocityRotations(){
        return targetDriveVelocityRotations;
    }

    //debug only
    public double getDriveVelocityRotations(){
        return getDriveVelocity() * cfg.rotationsPerMeter;
    }

    //debug only
    public double getVelocityError(){
        return Math.abs(getTargetDriveVelocity()) - Math.abs(getDriveVelocity());
    }

    //*Wrapping code from sds example swerve library
    public void setCommand(double steerRotations, double driveVelocity){
        targetSteerRotations = steerRotations;
        targetDriveVelocity = driveVelocity;

        /* From FRC 900's whitepaper, we add a cosine compensator to the applied drive velocity */
        /* To reduce the "skew" that occurs when changing direction */
        double steerMotorError = steerRotations - getSteerRotations();
        /* If error is close to 0 rotations, we're already there, so apply full power */
        /* If the error is close to 0.25 rotations, then we're 90 degrees, so movement doesn't help us at all */
        double cosineScalar = Math.cos(Units.rotationsToRadians(steerMotorError));
        /* Make sure we don't invert our drive, even though we shouldn't ever target over 90 degrees anyway */
        if (cosineScalar < 0.0) {
            cosineScalar = 0.0;
        }
        driveVelocity *= cosineScalar;

        // /* Back out the expected shimmy the drive motor will see */
        // /* Find the angular rate to determine what to back out */
        // double azimuthTurnRps = m_steerVelocity.getValue();
        // /* Azimuth turn rate multiplied by coupling ratio provides back-out rps */
        // double driveRateBackOut = azimuthTurnRps * m_couplingRatioDriveRotorToCANcoder;
        // velocityToSet -= driveRateBackOut;
        
        //alpha is a multilier based on the speed of steer velocity to offset the coupling in the wheel speed when turning
        double alpha = 0.25;
        // //multiply the alpha and steer velocity and then add it to drive velocity in order to offset
        double driveOffset = getSteerVelocity() * alpha;
        driveVelocity += driveOffset;

        setDriveVelocity(driveVelocity);
        setSteerRotations(steerRotations);
    }

    // Sets the velocity for the drive motor(s in meters per second.
    public void setDriveVelocity(double driveVelocity)
    {
        //line below is Debug only
        targetDriveVelocityRotations = driveVelocity * cfg.rotationsPerMeter;
        driveMotor.setControl(driveVelocityVoltage.withVelocity((driveVelocity * cfg.rotationsPerMeter)));
    }

    // setSteerAngle in radians
    public void setSteerRotations(double steerRotations)
    {
        steerMotor.setControl(steerPositionVoltage.withPosition(steerRotations));
    }

    /**Sets motors in the module to brake or coast mode
     * 
     * @param brake a boolean to indicate if motors should be in brake mode or not
     */
    public void setDriveMotorBraking(boolean brake)
    {
        if(brake)
        {
            steerMotor.setNeutralMode(NeutralModeValue.Brake);
            driveMotor.setNeutralMode(NeutralModeValue.Brake);
        }
        else
        {
            steerMotor.setNeutralMode(NeutralModeValue.Coast);
            driveMotor.setNeutralMode(NeutralModeValue.Coast);
        }
    }

    // configures motors with PIDF values, if it is inverted or not, current limits, etc.
    public void configureHardware()
    {
        MagnetSensorConfigs mgSenseCfg = new MagnetSensorConfigs();
        var error = steerEncoder.getConfigurator().refresh(mgSenseCfg, 0.5);
        if (!error.isOK()) {
            System.err.println(String.format("ERROR: SwerveModule %d steerEncoder response: %s ", cfg.moduleNumber, error.getDescription()));
        }
        System.out.println(String.format("SwerveModule %d Magnet MagnetOffset: %f", cfg.moduleNumber, mgSenseCfg.MagnetOffset));

        // Steer motor: accumulate every setting into a single configuration object and apply it once,
        // so unrelated fields don't get silently reset to defaults by later partial applies.
        TalonFXConfiguration steerConfigs = new TalonFXConfiguration();
        steerConfigs.TorqueCurrent.PeakForwardTorqueCurrent = cfg.steerCurrentLimit;
        steerConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -cfg.steerCurrentLimit;
        steerConfigs.Voltage.PeakForwardVoltage = cfg.steerVoltageLimit;
        steerConfigs.Voltage.PeakReverseVoltage = -cfg.steerVoltageLimit;
        steerConfigs.CurrentLimits.SupplyCurrentLimit = cfg.steerCurrentLimit;
        steerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        steerConfigs.Feedback.FeedbackRemoteSensorID = idcfg.steerEncoderID;
        steerConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        steerConfigs.Feedback.RotorToSensorRatio = cfg.steerGearRatio;
        steerConfigs.Feedback.SensorToMechanismRatio = 1.0;  // This should be used for remote CANCoder with continuous wrap.
        steerConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        steerConfigs.ClosedLoopGeneral.ContinuousWrap = true;
        steerConfigs.Slot0.kP = cfg.steerP;
        steerConfigs.Slot0.kI = cfg.steerI;
        steerConfigs.Slot0.kD = cfg.steerD;
        steerConfigs.Slot0.kV = cfg.steerV;
        steerConfigs.Slot0.kS = cfg.steerS;

        error = steerMotor.getConfigurator().apply(steerConfigs, 1.0);
        if (!error.isOK())
        {
            System.err.println(String.format("SwerveModule %d Steer Motor Configuration Error: %s ", cfg.moduleNumber, error.getDescription()));
        }

        error = steerMotor.setNeutralMode(NeutralModeValue.Brake, 1.0);
        if (!error.isOK()) {
            System.err.println(String.format("SwerveModule %d Steer Neutral Mode Error: %s", cfg.moduleNumber, error.getDescription()));
        }

        // Drive motor: same single-apply pattern.
        TalonFXConfiguration driveConfigs = new TalonFXConfiguration();
        driveConfigs.TorqueCurrent.PeakForwardTorqueCurrent = cfg.driveCurrentLimit;
        driveConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -cfg.driveCurrentLimit;
        driveConfigs.Voltage.PeakForwardVoltage = cfg.driveVoltageLimit;
        driveConfigs.Voltage.PeakReverseVoltage = -cfg.driveVoltageLimit;
        driveConfigs.CurrentLimits.SupplyCurrentLimit = cfg.driveCurrentLimit;
        driveConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        driveConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        driveConfigs.Slot0.kP = cfg.driveP;
        driveConfigs.Slot0.kI = cfg.driveI;
        driveConfigs.Slot0.kD = cfg.driveD;
        driveConfigs.Slot0.kV = cfg.driveV;
        driveConfigs.Slot0.kA = cfg.driveA;
        driveConfigs.Slot0.kS = cfg.driveS;

        error = driveMotor.getConfigurator().apply(driveConfigs, 1.0);
        if (!error.isOK())
        {
            System.err.println(String.format("SwerveModule %d Drive Motor Configuration Error: %s ", cfg.moduleNumber, error.getDescription()));
        }

        error = driveMotor.setNeutralMode(NeutralModeValue.Brake, 1.0);
        if (!error.isOK()) {
            System.err.println(String.format("SwerveModule %d Drive Neutral Mode Error: %s", cfg.moduleNumber, error.getDescription()));
        }

        driveMotor.setPosition(0);

        System.out.println(String.format("SwerveModule %d configured.", cfg.moduleNumber));
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        // Removed for elims:
        // builder.setSmartDashboardType(String.format("SwerveModule[%d]", cfg.moduleNumber));
        // builder.addDoubleProperty(String.format("Target Steer R %d", cfg.moduleNumber), this::getTargetSteerRotations, null);
        // builder.addDoubleProperty(String.format("Target Drive V %d", cfg.moduleNumber), this::getTargetDriveVelocity, null);
        // builder.addDoubleProperty(String.format("Steer R %d", cfg.moduleNumber), this::getSteerRotations, null);
        // builder.addDoubleProperty(String.format("Drive V %d", cfg.moduleNumber), this::getDriveVelocity, null);
        // builder.addDoubleProperty(String.format("Drive Position %d", cfg.moduleNumber), this::getDrivePosition, null);
        // builder.addDoubleProperty(String.format("Target Drive V Rotations %d", cfg.moduleNumber), this::getTargetDriveVelocityRotations, null);
        // builder.addDoubleProperty(String.format("Drive V Rotations %d", cfg.moduleNumber), this::getDriveVelocityRotations, null);
        // builder.addDoubleProperty(String.format("Drive V Error %d", cfg.moduleNumber), this::getVelocityError, null);
    //   steerEncoder.initSendable(builder);
    //   steerMotor.initSendable(builder);
    //   driveMotor.initSendable(builder);
    }

    /**
     * Gets the name of this Subsystem.
     *
     * @return Name
     */
    public String getName() {
        return SendableRegistry.getName(this);
    }

    /**
     * Sets the name of this Subsystem.
     *
     * @param name name
     */
    public void setName(String name) {
        SendableRegistry.setName(this, name);
    }

    /**Sets the percent output velocity to power
     * 
     * @param power the percentage the motor should operate at
     */
    public void setDebugTranslate(double power)
    {
        driveMotor.setControl(new DutyCycleOut(power));
    }

    /**Sets the percent output velocity of wheel angle to power
     * 
     * @param power the percentage the motor should operate at
     */
    public void setDebugRotate(double power)
    {
        steerMotor.setControl(new DutyCycleOut(power));
    }

    public double getLoad() {
        return Math.abs(driveMotor.getTorqueCurrent(true).getValueAsDouble());
    }

    public double getSteerVelocity(){
        steerEncoder.getVelocity().refresh();
        return steerEncoder.getVelocity().getValueAsDouble();
    }
}
