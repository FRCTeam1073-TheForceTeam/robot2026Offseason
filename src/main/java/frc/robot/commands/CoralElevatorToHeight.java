// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralElevator;
import frc.robot.subsystems.OI;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralElevatorToHeight extends Command {
  CoralElevator elevator;
  int branchLevel;
  double velocity;
  double targetHeight = 0.0;//in metres
  boolean terminate = false;

  /** Creates a new CoralElevatorToHeight. */
  public CoralElevatorToHeight(CoralElevator elevator, int branchLevel, boolean terminate) {
    this.elevator = elevator;
    this.branchLevel = branchLevel;
    this.terminate = terminate;
    
    if(branchLevel == 1){
      targetHeight = 0.45;
    }
    else if(branchLevel == 2){
      targetHeight = 0.72;
    }
    else if (branchLevel == 3){
      targetHeight = 1.11;
    }
    else if (branchLevel == 4){
      targetHeight = 1.73;
    }
    else if (branchLevel == 5){ //low algae
      targetHeight = 0.35;
    }
    else if (branchLevel == 6){ //high algae
      targetHeight = 0.745;
    }
    else if (branchLevel == 7){ //barge
      targetHeight = 1.84;
    }
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // velocity = (targetHeight - elevator.getPosition()) * 0.6;
    // if(targetHeight > elevator.getPosition()){
    //   velocity = MathUtil.clamp(velocity, 3, 12);
    // }
    // else{
    //   velocity = MathUtil.clamp(velocity, -12, -3);  
    // }
    // elevator.setVelocity(velocity);

    elevator.setPosition(targetHeight);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(terminate) {
      return Math.abs(elevator.getMeters() - targetHeight) < 0.02;
    }
    else {
      return false;
    }
  }
}
