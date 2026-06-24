package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.OI;

public class ClimberTeleop extends Command
{
    Climber climber;
    OI oi;
    private double velocity;

  public ClimberTeleop(Climber climber, OI oi) 
  {
    this.climber = climber;
    this.oi = oi;
    addRequirements(climber);
  }

  @Override
  public void initialize() 
  {

  }

  @Override
  public void execute() 
  {
    velocity = 0;
//     if (oi.getOperatorLeftBumper()){
//       velocity = 10;
//     }

//     else if (oi.getOperatorRightBumper()){
//       velocity = -10;
//     }
//     else{
//       velocity = 0;
//     }
    climber.setCommandedVelocity(velocity);
  }

  @Override
  public void end(boolean interrupted) 
  {

  }

  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
