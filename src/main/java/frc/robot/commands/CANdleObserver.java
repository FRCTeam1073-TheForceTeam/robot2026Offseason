// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANdleControl;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandStates;
import frc.robot.subsystems.CoralEndeffector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OI;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CANdleObserver extends Command {
  
  CANdleControl candleControl;
  CoralEndeffector endeffector;
  Climber climber;
  OI oi;
  CommandStates state;
  Drivetrain drivetrain;
  int numPerStrip;
  int numTotalLED;
  int candleNum;

  private boolean defenseMode;

  public CANdleObserver(CANdleControl CandleControl, CoralEndeffector Endeffector, Climber Climber, OI Oi, CommandStates state, Drivetrain drivetrain) {
    candleControl = CandleControl;
    endeffector = Endeffector;
    climber = Climber;
    oi = Oi;
    this.state = state;
    this.drivetrain = drivetrain;
    numPerStrip = candleControl.getStripLED();
    numTotalLED = candleControl.getTotalLED();
    candleNum = candleControl.getCandleNum();
    defenseMode = false;
    addRequirements(candleControl);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    candleControl.clearAnim();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // if (endeffector.getHasReef()){
    //   candleControl.setRGB(0, 255, 0, candleNum, numPerStrip + 3);//elevator forward - green
    // }
    // else{
    //   candleControl.setRGB(255, 0, 0, candleNum, numPerStrip + 3);//elevator forward - red
    // }

    if(defenseMode) {
      candleControl.setRGB(255, 255, 0, 8, 83); //set to purple
    }
    else {
      if (endeffector.getHasCoral()){
        candleControl.setRGB(255, 0, 0, candleNum, 15);//sides of funnel - light on
      }
      else{
        candleControl.setRGB(0, 0, 0, candleNum, 15);//sides of funnel - light off
      }
  
      if (RobotController.getBatteryVoltage() > 12){
        candleControl.setRGB(0, 255, 0, 0, candleNum);//CANdle - green
      }
      else if(RobotController.getBatteryVoltage() > 10){
        candleControl.setRGB(128, 128, 0, 0, candleNum);//CANdle - yellow
      }
      else{
        candleControl.setRGB(255, 0, 0, 0, candleNum);//CANdle - red
      }
  
      if (climber.getIsDisengaged()){
        candleControl.setRGB(0, 0, 255, candleNum + numPerStrip + 3, numPerStrip + 2);//elevator side - blue
      }
      else if (climber.getIsEngaged()){
        candleControl.setRGB(245, 146, 0, candleNum + numPerStrip + 3, numPerStrip + 2);//elevator side - orange
      }
      else if (climber.getIsAtZero()){
        candleControl.setRGB(150, 0, 255, candleNum + numPerStrip + 3, numPerStrip + 2);//elevator side - purple
      }
      else{
        candleControl.setRGB(128, 128, 128, candleNum + numPerStrip + 3, numPerStrip + 2);//elevator side - grey
      }
  
      if((state.getIsLidarAligning() || state.getIsLocalAligning()) && drivetrain.getAverageLoad() > 65) {
          candleControl.setRGB(255, 0, 0, 8, 83);
      }
    }

    // if(oi.getOperatorZeroFloorMech()) {
    //   defenseMode = !defenseMode;
    // }
    

    SmartDashboard.putBoolean("is Disengaged", climber.getIsDisengaged());
    SmartDashboard.putBoolean("is Engaged", climber.getIsEngaged());
    SmartDashboard.putBoolean("is zero", climber.getIsAtZero());

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