package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

public class TunerConstants {
    public static final String kCANBus = "Canivore";
    public static final int kPigeonId = 5;

    public static final SwerveDrivetrainConstants DrivetrainConstants =
    new SwerveDrivetrainConstants()
    .withCANBusName(kCANBus)
    .withPigeon2Id(kPigeonId)
    .withPigeon2Configs(new Pigeon2Configuration());

    // Physical constants from legacy SwerveModuleConfig
    private static final double kDriveGearRatio = 6.03;
    private static final double kSteerGearRatio = 287.0 / 11.0;
    private static final double kWheelRadiusMeters = 0.1016 / 2.0; // diameter 0.1016 m -> radius
    private static final double kSteerCurrentLimit = 25;
    private static final double kDriveCurrentLimit = 40;
    private static final double kSteerVoltageLimit = 8.5;
    private static final double kDriveVoltageLimit = 8.5;

    // PID gains from legacy SwerveModuleConfig
    private static final com.ctre.phoenix6.configs.Slot0Configs steerGains =
    new com.ctre.phoenix6.configs.Slot0Configs()
    .withKP(11.0).withKI(0.8).withKD(0.04).withKV(0.153).withKS(0.04);

    private static final com.ctre.phoenix6.configs.Slot0Configs driveGains =
    new com.ctre.phoenix6.configs.Slot0Configs()
    .withKP(0.35).withKI(0.0).withKD(0.0).withKV(0.12).withKA(0.0).withKS(0.015);

    // Preserve legacy voltage/current limit behavior
    private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration();
    static {
        steerInitialConfigs.CurrentLimits.SupplyCurrentLimit = kSteerCurrentLimit;
        steerInitialConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        steerInitialConfigs.Voltage.PeakForwardVoltage = kSteerVoltageLimit;
        steerInitialConfigs.Voltage.PeakReverseVoltage = -kSteerVoltageLimit;
    }

    private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
    static {
        driveInitialConfigs.CurrentLimits.SupplyCurrentLimit = kDriveCurrentLimit;
        driveInitialConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveInitialConfigs.Voltage.PeakForwardVoltage = kDriveVoltageLimit;
        driveInitialConfigs.Voltage.PeakReverseVoltage = -kDriveVoltageLimit;
    }

    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
    ConstantCreator = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
    .withDriveMotorGearRatio(kDriveGearRatio)
    .withSteerMotorGearRatio(kSteerGearRatio)
    .withWheelRadius(kWheelRadiusMeters)
    .withSteerMotorGains(steerGains)
    .withDriveMotorGains(driveGains)
    .withSteerMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
    .withDriveMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
    .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.RemoteCANcoder)
    .withCouplingGearRatio(0)
    .withSlipCurrent(kDriveCurrentLimit)
    .withSteerMotorInitialConfigs(steerInitialConfigs)
    .withDriveMotorInitialConfigs(driveInitialConfigs);

    // CANcoder offsets from magnet-offsets.md
    // Module numbering: 0=FL, 1=FR, 2=BL, 3=BR
    public static final SwerveModuleConstants<?, ?, ?> FrontLeft = ConstantCreator.createModuleConstants(
        8, 7, 6, -0.25927734375, 0.276, 0.276, false, false, false);

    public static final SwerveModuleConstants<?, ?, ?> FrontRight = ConstantCreator.createModuleConstants(
        11, 10, 9, -0.188232421875, 0.276, -0.276, false, false, false);

    public static final SwerveModuleConstants<?, ?, ?> BackLeft = ConstantCreator.createModuleConstants(
        14, 13, 12, -0.081298828125, -0.276, 0.276, false, false, false);

    public static final SwerveModuleConstants<?, ?, ?> BackRight = ConstantCreator.createModuleConstants(
        17, 16, 15, -0.157470703125, -0.276, -0.276, false, false, false);

    public static final double kMaxLinearSpeedMps = 4.75;
}
