// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralElevator;
import frc.robot.subsystems.OI;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralElevatorTeleop extends Command {

  OI oi;
  CoralElevator elevator;
  private double velocity;

  /** Creates a new CoralElevatorTeleop. */
  public CoralElevatorTeleop(CoralElevator elevator, OI oi){
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    this.oi = oi;
    addRequirements(elevator);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //elevator.setBrakeMode(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    velocity = oi.getOperatorLeftY() * 16.0;//TODO change controls
    elevator.setVelocity(velocity);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
