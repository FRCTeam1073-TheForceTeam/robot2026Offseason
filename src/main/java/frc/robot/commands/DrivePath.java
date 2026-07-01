// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Localizer;
import frc.robot.utilities.DashboardNames;

public class DrivePath extends Command
{
  private boolean quit;

  private final Drivetrain drivetrain;
  private final Localizer localizer;

  private final Optional<Trajectory<SwerveSample>> trajectory;
  private Optional<SwerveSample> currentSample;

  private Pose2d robotPose;

  private final PIDController xController = new PIDController(5.5, 0, 0.3);
  private final PIDController yController = new PIDController(5.5, 0, 0.3);
  private final PIDController thetaController = new PIDController(2.5, 0.0, 0.3);

  private double currentTime;
  private double startTime;
  private double endTime;

  private double maxVelocity;
  private double maxAngularVelocity;
  private double xVelocity;
  private double yVelocity;
  private double thetaVelocity;

  public DrivePath(Drivetrain drivetrain, Localizer localizer, Optional<Trajectory<SwerveSample>> trajectory)
  {
    this.drivetrain = drivetrain;
    this.localizer = localizer;
    this.trajectory = trajectory;

    quit = false;
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    SmartDashboard.putString(DashboardNames.DRIVE_PATH_STATUS.getKey(), "Idle");
    addRequirements(drivetrain);
  }

  @Override
  public void initialize()
  {
    SmartDashboard.putBoolean(DashboardNames.DRIVE_HAS_TRAJECTORY.getKey(), trajectory.isPresent());

    startTime = Timer.getFPGATimestamp();

    if (trajectory.isPresent()) {
      // get the initial time and pose of the robot
      endTime = trajectory.get().getTotalTime();
      robotPose = localizer.getPose();

      // if the current position is far from the start position quit
      Optional<Pose2d> initPose = trajectory.get().getInitialPose(isRedAlliance());
      if (initPose.isPresent()) {
        Transform2d diff = robotPose.minus(initPose.get());
        if (diff.getTranslation().getNorm() >= 2.0) {
          quit = true;
        }
      }
    }

    currentTime = 0.01;

    xController.reset();
    yController.reset();
    thetaController.reset();
  }

  @Override
  public void execute()
  {
    // get the start time and position
    currentTime = Timer.getFPGATimestamp() - startTime;
    robotPose = localizer.getPose();

    SmartDashboard.putNumber(DashboardNames.DRIVE_PATH_CURRENT_TIME.getKey(), currentTime);
    SmartDashboard.putBoolean(DashboardNames.DRIVE_PATH_TRAJECTORY.getKey(), trajectory.isPresent());

    if (trajectory.isPresent()) {
      Trajectory<SwerveSample> traj = trajectory.get();

      // fetch current sample
      currentSample = traj.sampleAt(currentTime, isRedAlliance());

      SmartDashboard.putBoolean(DashboardNames.DRIVE_PATH_CURRENT_SAMPLE.getKey(), currentSample.isPresent());

      if (currentSample.isPresent()) {
        SwerveSample trajSample = currentSample.get();
        ChassisSpeeds trajectorySpeeds = trajSample.getChassisSpeeds();
        maxVelocity = 2.5;
        maxAngularVelocity = 2.5;

        // v = PID(Transform + Robot_Pose) + Forward_Velocity * alpha
        // velocity = feedback + feedforward
        xVelocity = xController.calculate(robotPose.getX(), trajSample.x) + trajectorySpeeds.vxMetersPerSecond;
        yVelocity = yController.calculate(robotPose.getY(), trajSample.y) + trajectorySpeeds.vyMetersPerSecond;
        thetaVelocity = thetaController.calculate(robotPose.getRotation().getRadians(), trajSample.heading) + trajectorySpeeds.omegaRadiansPerSecond;

        xVelocity = MathUtil.clamp(xVelocity, -maxVelocity, maxVelocity);
        yVelocity = MathUtil.clamp(yVelocity, -maxVelocity, maxVelocity);
        thetaVelocity = MathUtil.clamp(thetaVelocity, -maxAngularVelocity, maxAngularVelocity);

        SmartDashboard.putNumber(DashboardNames.DRIVE_PATH_TARGET_X.getKey(), trajectorySpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber(DashboardNames.DRIVE_PATH_TARGET_Y.getKey(), trajectorySpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber(DashboardNames.DRIVE_PATH_TARGET_THETA.getKey(), trajSample.getPose().getRotation().getRadians());

        SmartDashboard.putNumber(DashboardNames.DRIVE_PATH_MAX_VELOCITY.getKey(), maxVelocity);

        SmartDashboard.putNumber(DashboardNames.DRIVE_PATH_COMMANDED_VX.getKey(), xVelocity);
        SmartDashboard.putNumber(DashboardNames.DRIVE_PATH_COMMANDED_VY.getKey(), yVelocity);
        SmartDashboard.putNumber(DashboardNames.DRIVE_PATH_COMMANDED_VW.getKey(), thetaVelocity);

        // set a field relative chassis speed
        drivetrain.setTargetChassisSpeeds(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xVelocity,
                yVelocity,
                thetaVelocity,
                localizer.getPose().getRotation()));
      } else {
        System.err.println("DrivePath No Sample Found");
        drivetrain.setTargetChassisSpeeds(new ChassisSpeeds(0, 0, 0));
        quit = true;
      }
    } else {
      SmartDashboard.putString(DashboardNames.DRIVE_PATH_STATUS.getKey(), "No Trajectory Found");
      System.err.println("DrivePath No Trajectory Found");
      quit = true;
    }
  }

  @Override
  public void end(boolean interrupted)
  {
    SmartDashboard.putBoolean(DashboardNames.DRIVE_PATH_END.getKey(), true);
    drivetrain.setTargetChassisSpeeds(new ChassisSpeeds(0, 0, 0));
  }

  @Override
  public boolean isFinished()
  {
    SmartDashboard.putBoolean(DashboardNames.DRIVE_PATH_PAST_TIME.getKey(), currentTime >= endTime);
    SmartDashboard.putBoolean(DashboardNames.DRIVE_PATH_QUIT.getKey(), quit);
    if (currentTime >= endTime || quit) {
      SmartDashboard.putString(DashboardNames.DRIVE_PATH_STATUS.getKey(), "Finished");
      System.out.println("IsFinishedRun");
      return true;
    }
    return false;
  }

  private boolean isRedAlliance()
  {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    return alliance == Alliance.Red;
  }
}
