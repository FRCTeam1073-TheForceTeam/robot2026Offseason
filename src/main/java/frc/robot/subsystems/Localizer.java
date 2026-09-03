// LOCALIZER: accesses drivetrain for odometry and AprilTagFinder for vision measurements

package frc.robot.subsystems;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.AprilTagFinder.VisionMeasurement;
import frc.robot.utilities.DashboardNames;

public class Localizer extends SubsystemBase
{
    // Between 0.0 and 1.0: 1.0 is unfiltered, 0.0 is no updates.
    private static final double velocityFilterAlpha = 0.7;

    private final Drivetrain driveTrain;
    private final AprilTagFinder finder;

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

    public void resetPose(Pose2d newPos)
    {
        driveTrain.resetOdometry(newPos);
    }

    public void resetOrientation()
    {
        driveTrain.resetRotation(new Rotation2d(0));
    }

    @Override
    public void periodic()
    {
        processVisionMeasurements();

        // Cache output:
        pose = driveTrain.getOdometry();
        // Compute speeds in field coordinates:
        ChassisSpeeds robotSpeeds = driveTrain.getChassisSpeeds();
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, pose.getRotation());

        // Simplistic IIR update of reported field-centric speeds:
        speeds = new ChassisSpeeds(
            (1.0 - velocityFilterAlpha) * speeds.vxMetersPerSecond + velocityFilterAlpha * fieldSpeeds.vxMetersPerSecond,
            (1.0 - velocityFilterAlpha) * speeds.vyMetersPerSecond + velocityFilterAlpha * fieldSpeeds.vyMetersPerSecond,
            (1.0 - velocityFilterAlpha) * speeds.omegaRadiansPerSecond + velocityFilterAlpha * fieldSpeeds.omegaRadiansPerSecond
        );

        Logger.recordOutput("Localizer/Pose", pose);
        Logger.recordOutput("Localizer/Speeds", speeds);
        Logger.recordOutput("Localizer/MeasurementCounter", measurementCounter);

        if (counter >= 50) {
            SmartDashboard.putNumber(DashboardNames.LOCALIZER_PS.getKey(), measurementCounter);
            measurementCounter = 0;
            counter = 0;
        } else {
            counter = counter + 1;
        }
    }

    private void processVisionMeasurements()
    {
        double now = Timer.getFPGATimestamp();
        double timeSinceLastUpdate = now - lastUpdateTime;
        boolean isTimeReady = timeSinceLastUpdate > timeGap;
        boolean isStable = measurementStable();

        Logger.recordOutput("Localizer/Vision/TimeSinceLastUpdate", timeSinceLastUpdate);
        Logger.recordOutput("Localizer/Vision/IsTimeReady", isTimeReady);
        Logger.recordOutput("Localizer/Vision/IsStable", isStable);

        List<VisionMeasurement> measurements = finder.getAllMeasurements();
        Logger.recordOutput("Localizer/Vision/MeasurementCount", measurements.size());

        if (isTimeReady && isStable) {
            for (int index = 0; index < measurements.size(); index++) {
                VisionMeasurement currentMeasurement = measurements.get(index);

                Logger.recordOutput("Localizer/Vision/Measurement_" + index + "/Pose", currentMeasurement.pose);
                Logger.recordOutput("Localizer/Vision/Measurement_" + index + "/TimeStamp", currentMeasurement.timeStamp);
                Logger.recordOutput("Localizer/Vision/Measurement_" + index + "/StddevX", currentMeasurement.stddevs[0]);
                Logger.recordOutput("Localizer/Vision/Measurement_" + index + "/StddevY", currentMeasurement.stddevs[1]);
                Logger.recordOutput("Localizer/Vision/Measurement_" + index + "/StddevTheta", currentMeasurement.stddevs[2]);

                driveTrain.addVisionMeasurement(currentMeasurement.pose, currentMeasurement.timeStamp, currentMeasurement.stddevs);
                measurementCounter++;
            }
            lastUpdateTime = now;

            finder.clearMeasurements();
            Logger.recordOutput("Localizer/Vision/MeasurementsProcessed", true);
        } else {
            Logger.recordOutput("Localizer/Vision/MeasurementsProcessed", false);
        }
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

        Logger.recordOutput("Localizer/Vision/LinearSpeed", linearSpeed);
        Logger.recordOutput("Localizer/Vision/LinearSpeedThreshold", linearSpeedThreshold);
        Logger.recordOutput("Localizer/Vision/AngularSpeed", angularSpeed);
        Logger.recordOutput("Localizer/Vision/AngularSpeedThreshold", angularSpeedThreshold);

        return (linearSpeed <= linearSpeedThreshold && angularSpeed <= angularSpeedThreshold);
    }
}
