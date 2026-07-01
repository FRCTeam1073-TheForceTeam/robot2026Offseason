// LOCALIZER: accesses drivetrain for odometry and AprilTagFinder for vision measurements

package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.AprilTagFinder.VisionMeasurement;

public class Localizer extends SubsystemBase
{
    // Between 0.0 and 1.0: 1.0 is unfiltered, 0.0 is no updates.
    private static final double velocityFilterAlpha = 0.7;

    private final Drivetrain driveTrain;
    private final AprilTagFinder finder;
    private final SwerveDriveKinematics kinematics;
    private SwerveDrivePoseEstimator estimator;

    private ChassisSpeeds speeds = new ChassisSpeeds(0.0, 0.0, 0.0); // Cached field centric velocity.
    private Pose2d pose = new Pose2d(); // Cached localized pose.

    private double lastUpdateTime;
    private int measurementCounter = 0;
    private int counter = 0;

    private double timeGap = 0.030;
    private double linearSpeedThreshold = 2.5;
    private double angularSpeedThreshold = 2;

    public Localizer(Drivetrain driveTrain, AprilTagFinder finder)
    {
        this.driveTrain = driveTrain;
        this.finder = finder;
        this.kinematics = driveTrain.getKinematics();

        estimator = new SwerveDrivePoseEstimator(
            kinematics, driveTrain.getOdometry().getRotation(), driveTrain.getSwerveModulePositions(), new Pose2d()
        );
        lastUpdateTime = Timer.getFPGATimestamp();
    }

    @Override
    public void initSendable(SendableBuilder builder)
    {
        builder.setSmartDashboardType("Localizer");
    }

    public double getTimeGap()
    {
        return timeGap;
    }

    public void setTimeGap(double time)
    {
        timeGap = time;
    }

    public double getLinearSpeedThreshold()
    {
        return linearSpeedThreshold;
    }

    public void setLinearSpeedThreshold(double speed)
    {
        linearSpeedThreshold = speed;
    }

    public double getAngularSpeedThreshold()
    {
        return angularSpeedThreshold;
    }

    public void setAngularSpeedThreshold(double angularSpeed)
    {
        angularSpeedThreshold = angularSpeed;
    }

    // creates an entirely new estimator so the rotation is reset for sure
    public void resetPose(Pose2d newPos)
    {
        estimator = new SwerveDrivePoseEstimator(kinematics, driveTrain.getGyroHeading(), driveTrain.getSwerveModulePositions(), newPos);
    }

    public void resetOrientation()
    {
        Pose2d resetPos = new Pose2d(estimator.getEstimatedPosition().getTranslation(), new Rotation2d(0));
        estimator = new SwerveDrivePoseEstimator(kinematics, driveTrain.getGyroHeading(), driveTrain.getSwerveModulePositions(), resetPos);
    }

    @Override
    public void periodic()
    {
        double now = Timer.getFPGATimestamp();
        estimator.updateWithTime(now, driveTrain.getGyroHeading(), driveTrain.getSwerveModulePositions());

        if (now - lastUpdateTime > timeGap && measurementStable()) {
            List<VisionMeasurement> measurements = finder.getAllMeasurements();
            for (int index = 0; index < measurements.size(); index++) {
                VisionMeasurement currentMeasurement = measurements.get(index);

                estimator.addVisionMeasurement(currentMeasurement.pose, currentMeasurement.timeStamp,
                    VecBuilder.fill(currentMeasurement.stddevs[0], currentMeasurement.stddevs[1], currentMeasurement.stddevs[2]));
                measurementCounter++;
            }
            lastUpdateTime = now;

            finder.clearMeasurements();
        }

        // Cache output:
        pose = estimator.getEstimatedPosition();
        // Compute speeds in field coordinates:
        ChassisSpeeds robotSpeeds = driveTrain.getChassisSpeeds();
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, estimator.getEstimatedPosition().getRotation());

        // Simplistic IIR update of reported field-centric speeds:
        speeds = new ChassisSpeeds(
            (1.0 - velocityFilterAlpha) * speeds.vxMetersPerSecond + velocityFilterAlpha * fieldSpeeds.vxMetersPerSecond,
            (1.0 - velocityFilterAlpha) * speeds.vyMetersPerSecond + velocityFilterAlpha * fieldSpeeds.vyMetersPerSecond,
            (1.0 - velocityFilterAlpha) * speeds.omegaRadiansPerSecond + velocityFilterAlpha * fieldSpeeds.omegaRadiansPerSecond
        );

        if (counter >= 50) {
            SmartDashboard.putNumber("Localizer/PS", measurementCounter);
            measurementCounter = 0;
            counter = 0;
        } else {
            counter = counter + 1;
        }
        // Update localized output for debug:
        SmartDashboard.putNumber("Localizer/Pose(x)", pose.getX());
        SmartDashboard.putNumber("Localizer/Pose(y)", pose.getY());
        SmartDashboard.putNumber("Localizer/Pose(q)", pose.getRotation().getRadians());
        SmartDashboard.putNumber("Localizer/Vel(x)", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Localizer/Vel(y)", speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Localizer/Vel(q)", speeds.omegaRadiansPerSecond);
        SmartDashboard.putNumber("Localizer/MC", measurementCounter);
    }

    // Returns field-centric, localizer based position estimate.
    public Pose2d getPose()
    {
        return pose;
    }

    // Returns field-centric, localizer based speeds.
    public ChassisSpeeds getSpeeds()
    {
        return speeds;
    }

    private boolean measurementStable()
    {
        ChassisSpeeds robotSpeeds = driveTrain.getChassisSpeeds();
        double linearSpeed = Math.sqrt(robotSpeeds.vxMetersPerSecond * robotSpeeds.vxMetersPerSecond
            + robotSpeeds.vyMetersPerSecond * robotSpeeds.vyMetersPerSecond);
        double angularSpeed = Math.abs(robotSpeeds.omegaRadiansPerSecond);
        return (linearSpeed <= linearSpeedThreshold && angularSpeed <= angularSpeedThreshold);
    }
}
