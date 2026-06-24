// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FloorPickupCollect;
import frc.robot.subsystems.FloorPickupPivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FloorScoreCoral extends Command {
  /** Creates a new floorEject. */
  FloorPickupCollect floorPickupCollect;
  FloorPickupPivot floorPickupPivot;

  double startTime;

  double scorePos = 3.0;

  public FloorScoreCoral(FloorPickupCollect floorPickupCollect, FloorPickupPivot floorPickupPivot) {
    this.floorPickupCollect = floorPickupCollect;
    this.floorPickupPivot = floorPickupPivot;

    addRequirements(floorPickupPivot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    floorPickupPivot.setRotatorPos(scorePos);
    if(Timer.getFPGATimestamp() - startTime > 0.5) {
      floorPickupCollect.setVelocity(-44);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Timer.getFPGATimestamp() - startTime > 3.0);
  }
}
