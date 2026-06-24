// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralEndeffector;
import frc.robot.subsystems.Drivetrain;

public class CreepToReef extends Command {
  public final double creepSpeed = 0.1;

  CoralEndeffector endeffector;
  Drivetrain drivetrain;
  int dir = -1; // Direction to creep.

  ChassisSpeeds speeds;
  int seenReef = 0;
  boolean hasReef = false;

  // TODO: Update when openMV is implemented
  /** This command creeps along the reef slowly until the sensor sees the reef then exits/ stops. 
   * Suggested by Cody.
  */
  public CreepToReef(Drivetrain drivetrain, CoralEndeffector coralEndeffector, int dir) {
    endeffector = coralEndeffector;
    speeds = new ChassisSpeeds(0.0,0.0, 0.0);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain, endeffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    seenReef = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    hasReef = endeffector.getHasReef();

    // Creep if we don't see the reef.
    if (!hasReef) {
      // Set our creep speed:
      speeds.vxMetersPerSecond = 0.02;
      speeds.vyMetersPerSecond = creepSpeed * dir;
      speeds.omegaRadiansPerSecond = 0.0;
      drivetrain.setTargetChassisSpeeds(speeds); // Stop moving.
    } else {
      speeds.vxMetersPerSecond = 0.0;
      speeds.vyMetersPerSecond = 0.0;
      speeds.omegaRadiansPerSecond = 0.0;
      drivetrain.setTargetChassisSpeeds(speeds); // Stop moving.
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    speeds.vxMetersPerSecond = 0.0;
    speeds.vyMetersPerSecond = 0.0;
    speeds.omegaRadiansPerSecond = 0.0;
    drivetrain.setTargetChassisSpeeds(speeds); // Stop moving.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Have to see the reef multiple times in a row.
    if (hasReef) {
      seenReef++;
    } else {
      if (seenReef > 0) seenReef--;
    }

    if (seenReef > 3) return true;
    else return false;
  }
}
