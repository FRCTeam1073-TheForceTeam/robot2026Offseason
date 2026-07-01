// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Localizer;

public class DriveStraight extends Command
{
  private final Drivetrain drivetrain;
  private final Localizer localizer;

  private double startTime = 0.0;

  public DriveStraight(Drivetrain drivetrain, Localizer localizer)
  {
    this.drivetrain = drivetrain;
    this.localizer = localizer;

    addRequirements(drivetrain, localizer);
  }

  @Override
  public void initialize()
  {
    startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void execute()
  {
    drivetrain.setTargetChassisSpeeds(
        ChassisSpeeds.fromFieldRelativeSpeeds(1.0, 0.0, 0.0, localizer.getPose().getRotation()));
  }

  @Override
  public void end(boolean interrupted)
  {
  }

  @Override
  public boolean isFinished()
  {
    if (Timer.getFPGATimestamp() - startTime >= 5.0) {
      return true;
    }
    return false;
  }
}
