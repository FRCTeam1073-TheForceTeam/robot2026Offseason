package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class CommandSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants driveTrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules) {
        super(TalonFX::new, TalonFX::new, CANcoder::new, driveTrainConstants, modules);
        CommandScheduler.getInstance().registerSubsystem(this);

        if (isSimulated()) {
            startSimThread();
        }
    }

    private void startSimThread() {
        m_lastSimTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        m_simNotifier = new Notifier(() -> {
                final double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
                double deltaTime = currentTime - m_lastSimTime;
                m_lastSimTime = currentTime;

                // use the measured time delta, get battery voltage from WPILib
                updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void close() {
        if (m_simNotifier != null) {
            m_simNotifier.close();
            m_simNotifier = null;
        }
        super.close();
    }

    private static boolean isSimulated() {
        return edu.wpi.first.wpilibj.RobotBase.isSimulation();
    }
}
