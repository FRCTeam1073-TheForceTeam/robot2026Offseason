package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.DashboardNames;


public class DumperBlocker extends SubsystemBase
{

    public static final int dumperBlockerMotorId = 31;

    public static final double gearRatio = 5.0;
    public static final double ampsPerNewtonMeter = 10.0;
    public static final double currentLimit = 10.0;
    public static final double hardstopCurrent = 8.0;

    public static final double extendVelocity = 2.0;
    public static final double retractVelocity = -2.0;
    public static final double zeroVelocity = 1.0;

    private final TalonFX dumperBlockerMotor;
    private final StatusSignal<AngularVelocity> velocitySig;
    private final StatusSignal<Current> currentSig;
    private final VelocityVoltage commandVelocityVoltage = new VelocityVoltage(0).withSlot(0);

    public DumperBlocker()
    {
        setName("DumperBlocker");

        dumperBlockerMotor = new TalonFX(dumperBlockerMotorId, new CANBus("rio"));
        velocitySig = dumperBlockerMotor.getVelocity();
        currentSig = dumperBlockerMotor.getTorqueCurrent();

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

        // Slot 0
        configs.Slot0.kV = 0.153;
        configs.Slot0.kP = 0.3;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.0;
        configs.Slot0.kA = 0.0;
        configs.Slot0.kS = 0.02;

        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        var status = dumperBlockerMotor.getConfigurator().apply(configs, 1.0);
        if (!status.isOK()) {
        System.err.println("Dumper Blocker: leader failed to config!");
        return false;
        }

        // Set our neutral mode to brake on:
        status = dumperBlockerMotor.setNeutralMode(NeutralModeValue.Brake, 1.0);
        if (!status.isOK()) {
        System.err.println("Dumper Blocker: neutral mode failed to config :(!");
        return false;
        }

        return true;
    }

    public void extend() {
        dumperBlockerMotor.setControl(
            commandVelocityVoltage.withVelocity(extendVelocity)
        );
    }

    public void retract() {
        dumperBlockerMotor.setControl(
            commandVelocityVoltage.withVelocity(retractVelocity)
        );
    }

    public void stop() {
        dumperBlockerMotor.setControl(new NeutralOut());
    }

    public void zero() {
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

    @Override
    public void periodic()
    {
        currentSig.refresh();
        velocitySig.refresh();

        torque = currentSig.getValueAsDouble() / ampsPerNewtonMeter;
        velocity = velocitySig.getValueAsDouble() * 2.0 * Math.PI / gearRatio;

        SmartDashboard.putNumber(DashboardNames.DUMPER_BLOCKER_VELOCITY.getKey(), getVelocityRadPerSec());

    }

}
