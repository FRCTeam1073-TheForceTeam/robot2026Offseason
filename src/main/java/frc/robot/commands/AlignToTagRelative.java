// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Localizer;
import frc.robot.subsystems.MapDisplay;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.CommandStates;

public class AlignToTagRelative extends Command 
{
  Drivetrain drivetrain;
  AprilTagFinder finder;
  CommandStates state;
  int aprilTagID;
  // Localizer localizer;
  // FieldMap fieldMap;
  // MapDisplay mapDisplay;
  int slot;
  boolean isRed = false;
  Pose2d targetLocation; // Last location we've seen the tag in ODOMETRY coordinates.
  Transform2d offset;
  Pose2d currentPose;
  PIDController xController;
  PIDController yController;
  PIDController thetaController;
  double xVelocity = 0.0;
  double yVelocity = 0.0;
  double wVelocity = 0.0;
  double xError = 0.0;
  double yError = 0.0;
  double wError = 0.0;
  int missCounter = 0;
  ChassisSpeeds speeds;


  private final static double maximumLinearVelocity = 1.7;   // Meters/second
  private final static double maximumRotationVelocity = 1.5; // Radians/second

  /** Creates a new alignToTag. */
  public AlignToTagRelative(Drivetrain drivetrain, AprilTagFinder finder, CommandStates state, int tagID, int slot) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.finder = finder; 
    this.state = state;
    this.slot = slot;
    this.currentPose = new Pose2d();
    aprilTagID = tagID;

    speeds = new ChassisSpeeds();

    xVelocity = 0;
    yVelocity = 0;
    wVelocity = 0;

    xController = new PIDController(
      2.5, 
      0.0, 
      0.03
    );

    yController = new PIDController(
      2.5, 
      0.0, 
      0.03
    );

    thetaController = new PIDController(
      2.0, 
      0.0,
      0.03
    );

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
    if(DriverStation.getAlliance().isPresent())
    {
      DriverStation.Alliance alliance = DriverStation.getAlliance().get();
      if (alliance == Alliance.Red)
      {
        isRed = true;
      }
      else
      {
        isRed = false;
      }
    }

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    SmartDashboard.putNumber("AlignToTagRelative/TagId", aprilTagID);
    
    double yOffset = 0.165;
    double endEffectorOffset = 0.1905;
    state.setIsLocalAligning(true);

    xController.reset();
    yController.reset();
    thetaController.reset();
    missCounter = 0;

    if (slot == 0)
    {
      offset = new Transform2d(0.42, endEffectorOffset, new Rotation2d(Math.PI));
    }
    else if (slot == -1)
    {
      offset = new Transform2d(0.42, -yOffset + endEffectorOffset, new Rotation2d(Math.PI));
    }
    else if (slot == 1)
    {
      offset = new Transform2d(0.42, yOffset + endEffectorOffset + 0.01, new Rotation2d(Math.PI));
    }
    
    // When we start, assume a large error until we update it.
    xError = 10;
    yError = 10;
    wError = 10;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    // Capture current pose for local *odometry* based movement.
    currentPose = drivetrain.getOdometry();

    // Drive on vision measurements.
    // Look for the tag in the currently tracked set:
    var tags = finder.getAllMeasurements();
    SmartDashboard.putNumber("AlignToTagRelative/Tags", tags.size());
    boolean found = false;
    for(int i = 0; i < tags.size(); i++){
      if (tags.get(i).tagID == aprilTagID) {
  
        // Update target location of tag in *odometry** coordinates with the offset
        // every time we see it.
        targetLocation = currentPose.plus(tags.get(i).relativePose.plus(offset));

        /// We didn't miss it, we have it.
        if (missCounter > 0) missCounter = 0;
        found = true;
      }
    }
    
    // Didn't find tag anywhere so we're keeping old target location but count misses.
    if (!found) missCounter++; // We missed seeing it this time, we're using old location?
    SmartDashboard.putNumber("AlignToTagRelative/MissCounter", missCounter);

    // We don't yet have a target location at all.
    if(targetLocation == null) {
      return;
    }

    // Update our error  target (in odo) from odo.
    xError = Math.abs(targetLocation.getX() - currentPose.getX());
    yError = Math.abs(targetLocation.getY() - currentPose.getY());
    wError = Math.abs(targetLocation.getRotation().getRadians() - currentPose.getRotation().getRadians());

    // Within ~1/10th second.
    if (missCounter < 60) {
      // We have the tag, align to the offset robot relative position (make the robot center 0,0,0 from the location)
      xVelocity = xController.calculate(currentPose.getX(), targetLocation.getX());
      yVelocity = yController.calculate(currentPose.getY(), targetLocation.getY());
      wVelocity = thetaController.calculate(currentPose.getRotation().getRadians(), targetLocation.getRotation().getRadians());

      xVelocity = MathUtil.clamp(xVelocity, -maximumLinearVelocity, maximumLinearVelocity);
      yVelocity = MathUtil.clamp(yVelocity, -maximumLinearVelocity, maximumLinearVelocity);
      wVelocity = MathUtil.clamp(wVelocity, -maximumRotationVelocity, maximumRotationVelocity);
      speeds.vxMetersPerSecond = xVelocity;
      speeds.vyMetersPerSecond = yVelocity;
      speeds.omegaRadiansPerSecond = wVelocity;

    } else {
      
      // Haven't seen target in too long => stop moving.
      speeds.vxMetersPerSecond = 0.0;
      speeds.vyMetersPerSecond = 0.0;
      speeds.omegaRadiansPerSecond = 0.0;
    }

    drivetrain.setTargetChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, currentPose.getRotation()));

    SmartDashboard.putNumber("AlignToTagRelative/TagId", aprilTagID);
    SmartDashboard.putNumber("AlignToTagRelative/ErrorX", xError);
    SmartDashboard.putNumber("AlignToTagRelative/ErrorY", yError);
    SmartDashboard.putNumber("AlignToTagRelative/ErrorW", wError);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    // Stop when we're interrupted.
    speeds.vxMetersPerSecond = 0.0;
    speeds.vyMetersPerSecond = 0.0;
    speeds.omegaRadiansPerSecond = 0.0;
    drivetrain.setTargetChassisSpeeds(speeds);

    // We have no target location
    targetLocation = null;
    state.setIsLocalAligning(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // We lost it for too long so we failed.
    if (missCounter >= 60) return true;

    // We're basically there so we succeeded.
    if (xError < 0.07 && yError < 0.05 && wError < 0.02)
      return true;
    else
      return false;
  }
}
