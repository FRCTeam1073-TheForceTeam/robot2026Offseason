package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.DashboardNames;


public class DumperBlocker extends SubsystemBase
{

    public static final int dumperBlockerMotorId = 31;

    public static final double gearRatio = 5.0;
    public static final double ampsPerNewtonMeter = 10.0;
    public static final double currentLimit = 10.0;
    public static final double hardstopCurrent = 5.5;

    public static final double extendVelocity = 5.0;
    public static final double retractVelocity = -5.0;
    public static final double zeroVelocity = 1.0;

    /** Volts required to hold the arm against gravity at horizontal, where its torque peaks. */
    public static final double gravityFeedforwardVolts = 0.5;

    /** Motion Magic profile limits, in rotor rotations per second and per second squared. */
    public static final double cruiseVelocity = 6.0;
    public static final double acceleration = 40.0;

    /**
     * Deployed setpoint. Gravity pulls the arm away from the top hardstop, so the stop carries
     * no load and driving into it only stalls the motor. Keep this just short of the stop so the
     * arm holds in free space against gravity alone.
     */
    public static final double deployedPosition = 1.20;
    public static final double stowedPosition = 0.0;

    /** Error below which the arm has arrived and drops to an open-loop gravity hold. */
    public static final double holdEnterTolerance = 0.03;
    /** Error above which the arm has drifted and the profile re-engages. */
    public static final double holdExitTolerance = 0.10;
    /** Rotor speed below which the arm counts as not moving, in rotations per second. */
    public static final double stallVelocity = 0.25;
    /** Loops of no motion before a blocked arm gives up and holds (~0.25 s at 50 Hz). */
    public static final int stallLoops = 12;

    public static final double minPositionRadians = 0.0;
    public static final double maxPositionRadians = 1.5;

    private enum Mode { NONE, VELOCITY, POSITION }
    private double targetVelocity = 0.0;
    private double targetPosition = 0.0;

    private final TalonFX dumperBlockerMotor;
    private final StatusSignal<AngularVelocity> velocitySig;
    private final StatusSignal<Angle> positionSig;
    private final StatusSignal<Current> currentSig;
    private final StatusSignal<Temperature> tempSig;
    private final StatusSignal<Current> supplyCurrentSig;
    private final StatusSignal<Current> statorCurrentSig;
    private final VelocityVoltage commandVelocityVoltage = new VelocityVoltage(0).withSlot(0);
    private final MotionMagicVoltage commandedMotionMagic = new MotionMagicVoltage(0).withSlot(1);
    private final VoltageOut commandedHoldVoltage = new VoltageOut(0);

    private boolean holding = false;
    private int stallCount = 0;

    private Mode mode = Mode.NONE;

    public DumperBlocker()
    {
        setName("DumperBlocker");

        dumperBlockerMotor = new TalonFX(dumperBlockerMotorId, new CANBus("rio"));
        velocitySig = dumperBlockerMotor.getVelocity();
        positionSig = dumperBlockerMotor.getPosition();
        currentSig = dumperBlockerMotor.getTorqueCurrent();
        tempSig = dumperBlockerMotor.getDeviceTemp();
        supplyCurrentSig = dumperBlockerMotor.getSupplyCurrent();
        statorCurrentSig = dumperBlockerMotor.getStatorCurrent();

        boolean hardwareConfigured = configureHardware();
        if (!hardwareConfigured) {
        System.err.println("Dumper Blocker: Hardware Failed To Configure!");
        }
        SmartDashboard.putBoolean(DashboardNames.DUMPER_BLOCKER_HW_CONFIGURED.getKey(), hardwareConfigured);
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
        configs.CurrentLimits.StatorCurrentLimit = 25.0;
        configs.CurrentLimits.StatorCurrentLimitEnable = true;

        // Slot 0 Velocity
        configs.Slot0.kV = 0.153;
        configs.Slot0.kP = 0.3;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.0;
        configs.Slot0.kA = 0.0;
        configs.Slot0.kS = 0.02;

        // Slot 1 Position
        configs.Slot1.kV = 0.153;
        configs.Slot1.kP = 7.0;
        configs.Slot1.kI = 0.0;
        configs.Slot1.kD = 0.01;
        configs.Slot1.kA = 0.0;
        configs.Slot1.kS = 0.02;
        // Gravity is compensated in periodic() via gravityFeedforward(), because position 0
        // is the arm straight down rather than horizontal, which is what Slot1.GravityType
        // (Arm_Cosine) would require. Leave the slot's kG at zero so it isn't applied twice.
        configs.Slot1.kG = 0.0;

        // Motion Magic generates a trapezoidal profile with a real deceleration phase and
        // applies the slot's kS/kV/kA along it, so the arm eases into the target instead of
        // arriving at full speed the way a slew-limited setpoint did.
        configs.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;
        configs.MotionMagic.MotionMagicAcceleration = acceleration;

        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        var status = dumperBlockerMotor.getConfigurator().apply(configs, 0.1);
        if (!status.isOK()) {
        System.err.println("Dumper Blocker: leader failed to config!");
        return false;
        }

        // Set our neutral mode to brake on:
        status = dumperBlockerMotor.setNeutralMode(NeutralModeValue.Brake, 0.1);
        if (!status.isOK()) {
        System.err.println("Dumper Blocker: neutral mode failed to config :(!");
        return false;
        }

        dumperBlockerMotor.setPosition(0);

        return true;
    }

    public void setVelocity(double radiansPerSecond) {
        mode = Mode.VELOCITY;
        targetVelocity = radiansPerSecond;
    }

    public void setPosition(double radians) {
        if (radians != targetPosition || mode != Mode.POSITION) {
            holding = false;
            stallCount = 0;
        }
        mode = Mode.POSITION;
        targetPosition = radians;
    }

    public boolean isHolding() {
        return holding;
    }

    public double getDeviceTempCelsius() {
        return tempSig.getValueAsDouble();
    }

    /** Amps drawn from the battery, which is what the 20 A breaker actually sees. */
    public double getSupplyCurrentAmps() {
        return supplyCurrentSig.getValueAsDouble();
    }

    /** Amps through the windings, which is what heats the motor. */
    public double getStatorCurrentAmps() {
        return statorCurrentSig.getValueAsDouble();
    }

    public void stop() {
        dumperBlockerMotor.setControl(new NeutralOut());
    }

    public void zero() {
        mode = Mode.VELOCITY;
        dumperBlockerMotor.setControl(
            commandVelocityVoltage.withVelocity(zeroVelocity)
        );
    }

    public void setZero() {
        dumperBlockerMotor.setPosition(0);
    }

    public double getTorqueCurrent() {
        return currentSig.getValueAsDouble() / ampsPerNewtonMeter;
    }

    public double getVelocityRadPerSec() {
        return velocitySig.getValueAsDouble() * 2.0 * Math.PI / gearRatio;
    }

    public double getPositionRadians() {
        return positionSig.getValueAsDouble();
    }

    /**
     * Volts of gravity compensation for a given arm position. Position 0 is the arm hanging
     * straight down, where gravity pulls along the arm and produces no torque about the pivot;
     * torque peaks a quarter turn later at horizontal. So the load scales with sin(angle).
     */
    private double gravityFeedforward(double positionRotations) {
        double mechanismRadians = positionRotations * 2.0 * Math.PI / gearRatio;
        return gravityFeedforwardVolts * Math.sin(mechanismRadians);
    }

    // public String getBrake() {
    //     return dumperBlockerMotor.;

    @Override
    public void periodic()
    {
        currentSig.refresh();
        velocitySig.refresh();
        positionSig.refresh();
        tempSig.refresh();
        supplyCurrentSig.refresh();
        statorCurrentSig.refresh();
    
        if (mode == Mode.POSITION) {
            double clampedCommand = MathUtil.clamp(targetPosition, minPositionRadians, maxPositionRadians);
            double gravityVolts = gravityFeedforward(getPositionRadians());
            double error = clampedCommand - getPositionRadians();

            stallCount = Math.abs(velocitySig.getValueAsDouble()) < stallVelocity ? stallCount + 1 : 0;

            // Once the arm has arrived - or has clearly run into something and stopped - drop the
            // closed loop and hold with gravity feedforward alone. Holding closed loop against a
            // target it cannot reach burns kP * error forever with nothing to show for it.
            if (holding) {
                if (Math.abs(error) > holdExitTolerance) {
                    holding = false;
                    stallCount = 0;
                }
            } else if (Math.abs(error) < holdEnterTolerance || stallCount >= stallLoops) {
                holding = true;
            }

            if (holding) {
                dumperBlockerMotor.setControl(commandedHoldVoltage.withOutput(gravityVolts));
            } else {
                dumperBlockerMotor.setControl(
                    commandedMotionMagic.withPosition(clampedCommand).withFeedForward(gravityVolts));
            }

            SmartDashboard.putNumber("DumperBlocker/clampedCommand", clampedCommand);
            SmartDashboard.putNumber("DumperBlocker/GravityFeedforward", gravityVolts);
            SmartDashboard.putBoolean("DumperBlocker/Holding", holding);
            SmartDashboard.putNumber("DumperBlocker/ProfileSetpoint",
                dumperBlockerMotor.getClosedLoopReference().getValueAsDouble());
        } 

        SmartDashboard.putNumber(DashboardNames.DUMPER_BLOCKER_VELOCITY.getKey(), getVelocityRadPerSec());
        SmartDashboard.putNumber(DashboardNames.DUMPER_BLOCKER_TORQUE_CURRENT.getKey(), getTorqueCurrent());
        // SmartDashboard.putString(DashboardNames.DUMPER_BLOCKER_BRAKEMODE.getKey(), getBrake());
        SmartDashboard.putNumber(DashboardNames.DUMPER_BLOCKER_POSITION.getKey(), getPositionRadians());
        SmartDashboard.putNumber("DumperBlocker/TargetPosition", targetPosition);
        SmartDashboard.putNumber(DashboardNames.DUMPER_BLOCKER_TEMP.getKey(), getDeviceTempCelsius());
        SmartDashboard.putNumber(DashboardNames.DUMPER_BLOCKER_SUPPLY_CURRENT.getKey(), getSupplyCurrentAmps());
        SmartDashboard.putNumber(DashboardNames.DUMPER_BLOCKER_STATOR_CURRENT.getKey(), getStatorCurrentAmps());

    }

}
