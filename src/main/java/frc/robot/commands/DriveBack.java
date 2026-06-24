// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralEndeffector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Localizer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveBack extends Command {
  Drivetrain drivetrain;
  Localizer localizer;

  /** Creates a new command. */
  public DriveBack(Drivetrain drivetrain, Localizer localizer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.localizer = localizer;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds backSpeed = new ChassisSpeeds(-1, 0, 0);
    drivetrain.setTargetChassisSpeeds(backSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     drivetrain.setTargetChassisSpeeds(
      new ChassisSpeeds(0, 0, 0)
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return CoralEndeffector.getHasCoral();
  }
}
