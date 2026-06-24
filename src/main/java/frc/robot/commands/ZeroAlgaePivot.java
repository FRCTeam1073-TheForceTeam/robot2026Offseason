// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.CoralEndeffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ZeroAlgaePivot extends Command {
  /** Creates a new ZeroAlgaePivot. */
  AlgaePivot algaePivot;
  int count;
  public ZeroAlgaePivot(AlgaePivot algaePivot) {
    this.algaePivot = algaePivot;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(algaePivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    count = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    algaePivot.setRotatorVel(-15);
    if(algaePivot.getRotatorLoad() > 5){
      count++;
    }
    else if(algaePivot.getRotatorLoad() <= 8 && count > 0){
      count--;
    }  
    // else{
    //   count = 0;
    // }
    SmartDashboard.putNumber("AlgaePivot/Count", count);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algaePivot.setRotatorVel(0);
    algaePivot.zeroRotator();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return (count > 3 || CoralEndeffector.getHasCoral());
  }
}
