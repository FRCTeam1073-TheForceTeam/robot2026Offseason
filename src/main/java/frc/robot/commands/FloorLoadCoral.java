// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FloorPickupCollect;
import frc.robot.subsystems.FloorPickupPivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FloorLoadCoral extends Command {
  /** Creates a new FloorIntake. */
  FloorPickupPivot floorPickupPivot;
  FloorPickupCollect floorPickupCollect;

  double pickUpPos = 20.60;
  double velocity;
  double timeStart;

  public FloorLoadCoral(FloorPickupPivot floorPickupPivot, FloorPickupCollect floorPickupCollect) {
    this.floorPickupPivot = floorPickupPivot;
    this.floorPickupCollect = floorPickupCollect;
    velocity = 0;

    addRequirements(floorPickupPivot, floorPickupCollect);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(floorPickupCollect.getLoad() <= 20) {
      velocity = 25;
    }
    else {

      velocity = 0;
    } 
    floorPickupCollect.setVelocity(velocity);
    floorPickupPivot.setRotatorPos(pickUpPos);
    if(floorPickupCollect.getLoad() <= 20) {
      velocity = 25;
    }
    floorPickupCollect.setVelocity(velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    floorPickupCollect.setVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
