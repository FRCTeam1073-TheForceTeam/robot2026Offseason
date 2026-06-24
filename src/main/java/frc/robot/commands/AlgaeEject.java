// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.CoralEndeffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeEject extends Command {
  /** Creates a new AlgaeEject. */
  CoralEndeffector coralEndeffector;
  double timeAtInit;
  AlgaePivot pivot;
  public AlgaeEject(CoralEndeffector coralEndeffector, AlgaePivot pivot) {
    this.pivot = pivot;
    this.coralEndeffector = coralEndeffector;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coralEndeffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeAtInit = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(pivot.getRotatorLoad() > 5){
      pivot.setRotatorVel(-6);
    }
    coralEndeffector.setVelocity(-15);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Timer.getFPGATimestamp() - timeAtInit > 2.5) {
      return true;
    }
    return false;
  }
}
