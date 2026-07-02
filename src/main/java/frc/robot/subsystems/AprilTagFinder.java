// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.DashboardNames;

public class AprilTagFinder extends SubsystemBase
{
  public static class VisionMeasurement
  {
    public final Pose2d pose; // this is in field coordinates
    public final Transform2d relativePose; // this is in robot coordinates.
    public final double timeStamp;
    public final int tagID;
    public final double[] stddevs; // Error in this measurement (standard deviations)

    public VisionMeasurement(Pose2d pose, Transform2d relativePose, double timeStamp, int tagID, double[] stddevs)
    {
      this.pose = pose;
      this.relativePose = relativePose;
      this.timeStamp = timeStamp;
      this.tagID = tagID;
      this.stddevs = stddevs;
    }
  }

  private static class RobotCamera
  {
    final PhotonCamera camera;
    final Transform3d transform;
    final boolean isTurret;

    RobotCamera(PhotonCamera camera, Transform3d transform)
    {
      this(camera, transform, false);
    }

    RobotCamera(PhotonCamera camera, Transform3d transform, boolean isTurret)
    {
      this.camera = camera;
      this.transform = transform;
      this.isTurret = isTurret;
    }
  }

  private final double ambiguityThreshold = 0.4;
  private boolean hasAprilTags;
  // Base stddevs for measurements:
  private final double[] baseStddevs = {0.5, 0.5, 0.5};
  // Ignore things farther away than this.
  private static final double maxRange = 4.0;

  private final List<VisionMeasurement> visionMeasurements = new ArrayList<>();
  private final Turret turret;
  private final Drivetrain drivetrain;
  private final List<RobotCamera> cameras = new ArrayList<>();
  private final List<PhotonPoseEstimator> estimators = new ArrayList<>();

  public AprilTagFinder(Turret turret, Drivetrain drivetrain)
  {
    this.turret = turret;
    this.drivetrain = drivetrain;

    System.out.println("Creating April Tag Object");
    //
    // These are poses in *ROBOT* coordinates:
    //
    // Robot coordinates have +X forward, +Y out left of robot and +Z up (opposite of gravity)
    // The origin of X,Y is the geometric center of the robot frame perimeter.
    // The origin of Z is *ON THE FLOOR* it is a virtual point at zero field height that is not actually *inside* the robot. It is the
    // projection of the X,Y geometric center onto the floor.
    //
    // Rotations are angles about these primary axes using the right-hand-rule.
    //
    // The turret height is to the center of the turret rotation bearing plane. Turret location is center of turret rotation bearing.
    //
    // Center of pigeon height is 4.75in, offset from X,Y center of robot ()
    //

    // We have coordinates from EM in "pigeon offset coordinates" not robot coordinates.
    Translation3d pigeonOffset = new Translation3d(Units.inchesToMeters(-1.0), Units.inchesToMeters(2.5), Units.inchesToMeters(4.75));

    cameras.add(new RobotCamera(new PhotonCamera("Left_Front"),
        new Transform3d(new Translation3d(Units.inchesToMeters(-8.977), Units.inchesToMeters(8.448), Units.inchesToMeters(5.152)).plus(pigeonOffset),
            new Rotation3d(0, Math.toRadians(-21), Math.toRadians(65)))));
    cameras.add(new RobotCamera(new PhotonCamera("Left_Back"),
        new Transform3d(new Translation3d(Units.inchesToMeters(-10.858), Units.inchesToMeters(7.855), Units.inchesToMeters(7.562)).plus(pigeonOffset),
            new Rotation3d(0, Math.toRadians(-21), Math.toRadians(150)))));
    cameras.add(new RobotCamera(new PhotonCamera("Right_Front"),
        new Transform3d(new Translation3d(Units.inchesToMeters(-8.977), Units.inchesToMeters(-13.448), Units.inchesToMeters(5.152)).plus(pigeonOffset),
            new Rotation3d(0, Math.toRadians(-21), Math.toRadians(-65)))));
    cameras.add(new RobotCamera(new PhotonCamera("Right_Back"),
        new Transform3d(new Translation3d(Units.inchesToMeters(-10.858), Units.inchesToMeters(-12.855), Units.inchesToMeters(7.652)).plus(pigeonOffset),
            new Rotation3d(0, Math.toRadians(-21), Math.toRadians(-150)))));
    cameras.add(new RobotCamera(new PhotonCamera("Turret"),
        new Transform3d(new Translation3d(Units.inchesToMeters(-3.47), Units.inchesToMeters(-7.51), Units.inchesToMeters(12.0)).plus(pigeonOffset),
            new Rotation3d(0, 0, 0)),
        true));

    for (RobotCamera camera : cameras) {
      estimators.add(new PhotonPoseEstimator(FieldMap.fieldMap, camera.transform));
    }
  }

  public List<VisionMeasurement> getAllMeasurements()
  {
    return visionMeasurements;
  }

  public List<PhotonTrackedTarget> getCamTargets(PhotonCamera camera)
  {
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    List<PhotonTrackedTarget> targets = new ArrayList<>();

    for (PhotonPipelineResult result : results) {
      if (result.hasTargets()) {
        targets.addAll(result.getTargets());
      }
    }
    return targets;
  }

  public Transform2d toTransform2d(Transform3d t3d)
  {
    return new Transform2d(t3d.getX(), t3d.getY(), t3d.getRotation().toRotation2d());
  }

  public List<VisionMeasurement> getCamMeasurements(List<PhotonPipelineResult> results, Transform3d camTransform3d)
  {
    List<VisionMeasurement> measurements = new ArrayList<>();
    for (PhotonPipelineResult result : results) {
      if (result.hasTargets()) {
        double resultTimestamp = result.getTimestampSeconds(); // Adjusted for each result for time compensation.

        for (PhotonTrackedTarget target : result.getTargets()) {
          Optional<Pose3d> tagPose = FieldMap.fieldMap.getTagPose(target.getFiducialId());
          if (tagPose.isPresent()) {
            if (target.getPoseAmbiguity() != -1 && target.getPoseAmbiguity() < ambiguityThreshold) {
              Transform3d best = target.getBestCameraToTarget();
              // Field coordinates:
              Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(best, tagPose.get(), camTransform3d.inverse());

              // In robot coordinates:
              Transform2d relativePose = toTransform2d(camTransform3d.plus(best));
              double range = relativePose.getTranslation().getNorm();

              // Ignore things that are too far away:
              if (range < maxRange) {
                // TODO: Estimated STD Deviations from Photon vision:
                double[] stdDevs = estimateStddevs(range, relativePose.getRotation().getRadians() + robotPose.getRotation().getZ());

                measurements.add(new VisionMeasurement(robotPose.toPose2d(), relativePose, resultTimestamp,
                    target.getFiducialId(), stdDevs));
              }
            }
          }
        } // End loop over targets.
      }
    }
    return measurements;
  }

  public Transform3d getRobotCamTransform(int index)
  {
    return cameras.get(index).transform;
  }

  public List<VisionMeasurement> getMultiTagEstimate(List<PhotonPipelineResult> results, PhotonPoseEstimator estimator, Transform3d camTransform3d)
  {
    estimator.setRobotToCameraTransform(camTransform3d);
    List<VisionMeasurement> measurements = new ArrayList<>();
    for (PhotonPipelineResult result : results) {
      // In field coordinates:
      Optional<EstimatedRobotPose> pose = estimator.estimateCoprocMultiTagPose(result);

      if (pose.isEmpty()) {
        // TODO: This seems like redundant work if we're doing multi tag poses?
        pose = estimator.estimateLowestAmbiguityPose(result);
        if (pose.isEmpty()) {
          continue;
        }
        if (pose.get().targetsUsed.isEmpty() || pose.get().targetsUsed.get(0).getPoseAmbiguity() > ambiguityThreshold) {
          continue;
        }
      }
      EstimatedRobotPose estimatedPose = pose.get();
      double minDist = 100.0;
      double minAngle = 0.0;
      for (PhotonTrackedTarget t : estimatedPose.targetsUsed) {
        Transform3d best = t.getBestCameraToTarget();
        double dist = best.getTranslation().getNorm();
        if (dist < minDist) {
          minDist = dist;
          minAngle = best.getRotation().getZ(); // Yaw angle.
        }
      }

      double[] stdDevs = estimateStddevs(minDist, minAngle); // TODO: find the actual value
      hasAprilTags = true;
      measurements.add(new VisionMeasurement(estimatedPose.estimatedPose.toPose2d(), new Transform2d(), estimatedPose.timestampSeconds, 0, stdDevs));
    }
    return measurements;
  }

  public void clearMeasurements()
  {
    visionMeasurements.clear();
  }

  @Override
  public void periodic()
  {
    hasAprilTags = false;
    double turretVelocity = turret.getVelocityRadPerSec();

    for (int i = 0; i < cameras.size(); i++) {
      RobotCamera cam = cameras.get(i);

      List<PhotonPipelineResult> results = cam.camera.getAllUnreadResults();
      Transform3d transform = cam.transform;

      // If the camera is the turrets camera, and the velocty of the turret is acceptable we will use it. And if not we will skip over using the camera.
      if (cam.isTurret) {
        // If the camera is the turret but it is not zeroed/indexed skip it.
        if (!turret.hasZero()) {
          SmartDashboard.putBoolean(DashboardNames.APRIL_TAG_FINDER_USING_TURRET_CAM.getKey(), false);
          continue;
        }

        // If the turret is moving too quickly then skip it otherwise try to use it.
        // TODO: Revisit threshold
        if (Math.abs(turretVelocity) < 1.0) {
          double totalLatencyMs = 0;
          double count = 0.0;
          for (PhotonPipelineResult result : results) {
            totalLatencyMs += result.metadata.getLatencyMillis();
            count = count + 1.0;
          }
          double averageLatencySec = 0.0;
          if (count > 0.0) {
            averageLatencySec = (totalLatencyMs / count) / 1000.0;
          }

          // Estimate turret angle at point of average latency from measurements:
          double turretAngle = turret.getPositionRadians() - turretVelocity * averageLatencySec; // TODO: Tweak this number
          transform = transform.plus(new Transform3d(new Translation3d(), new Rotation3d(0, 0, turretAngle)))
              .plus(new Transform3d(
                  new Translation3d(Units.inchesToMeters(-0.136), Units.inchesToMeters(-6.125), Units.inchesToMeters(5.187)),
                  new Rotation3d(0, Math.toRadians(-15), 0)));
          SmartDashboard.putBoolean(DashboardNames.APRIL_TAG_FINDER_USING_TURRET_CAM.getKey(), true);
        } else {
          SmartDashboard.putBoolean(DashboardNames.APRIL_TAG_FINDER_USING_TURRET_CAM.getKey(), false);
          continue; // Skip turret if it's moving too fast.
        }
      } // End camera is turret.

      List<VisionMeasurement> measurements = getCamMeasurements(results, transform);
      // estimator.addHeadingData(drivetrain.getPreviousUpdateTime(), drivetrain.getGyroHeading());
      // List<VisionMeasurement> measurements = getMultiTagEstimate(results, estimators.get(i), transform);
      visionMeasurements.addAll(measurements);
    }
    SmartDashboard.putBoolean(DashboardNames.APRIL_TAG_FINDER_HAS_TAGS.getKey(), hasAprilTags);
  }

  private double[] estimateStddevs(double range, double bearing)
  {
    double[] result = baseStddevs.clone();

    // TODO: Use bearing + range error model so that position errors are not symmertric.

    result[0] += 0.2 * range;
    result[1] += 0.2 * range;
    result[2] += 0.15 * range;
    return result;
  }
}
