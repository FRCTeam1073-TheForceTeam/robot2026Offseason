// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.print.attribute.standard.Fidelity;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.CommandStates;
import frc.robot.subsystems.CoralElevator;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FieldMap;
import frc.robot.subsystems.Lidar;
import frc.robot.subsystems.Localizer;
import frc.robot.subsystems.MapDisplay;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SmartAlign extends Command {
  /** Creates a new SmartAlign. */
  Drivetrain drivetrain;
  Localizer localizer;
  FieldMap fieldMap;
  MapDisplay mapDisplay; 
  AprilTagFinder aprilTagFinder; 
  Command smartAlignCommand;
  Lidar lidar;
  CoralElevator elevator;
  CommandStates state;
  int slot;
  int tagID;


  boolean isRed = false;
  public SmartAlign(Drivetrain drivetrain, Localizer localizer, CommandStates state, FieldMap fieldMap, MapDisplay mapDisplay, CoralElevator coralElevator, Lidar lidar, AprilTagFinder aprilTagFinder, int slot) {
    this.drivetrain = drivetrain;
    this.fieldMap = fieldMap;
    this.localizer = localizer;
    this.mapDisplay = mapDisplay;
    this.aprilTagFinder = aprilTagFinder;
    this.lidar = lidar;
    this.slot = slot;
    this.state = state;
    elevator = coralElevator;
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements();
  }

  public Command create(Drivetrain drivetrain, Localizer localizer, FieldMap fieldMap, MapDisplay mapDisplay, CoralElevator coralElevator, Lidar lidar, AprilTagFinder aprilTagFinder, int tagID, boolean isRed, int slot){
    return new SequentialCommandGroup(
      new AlignToTag(drivetrain, localizer, fieldMap, mapDisplay, state, true, tagID, slot),
      new AlignToTagRelative(drivetrain, aprilTagFinder, state, tagID, slot),
      new LidarAlign(lidar, drivetrain, state)
    );
    // return new ParallelRaceGroup(
    //   new CoralElevatorToHeight(coralElevator, 2, false),
    //   new SequentialCommandGroup(
    //     new AlignToTag(drivetrain, localizer, fieldMap, mapDisplay, true, tagID, slot),
    //     new AlignToTagRelative(drivetrain, aprilTagFinder, tagID, slot),
    //     new LidarAlign(lidar, drivetrain)
    //   )
    // );
  }

  public Command createSource(Drivetrain drivetrain, Localizer localizer, FieldMap fieldMap, MapDisplay mapDisplay, CoralElevator coralElevator, Lidar lidar, AprilTagFinder aprilTagFinder, int tagID, int slot){
      return new AlignToTag(drivetrain, localizer, fieldMap, mapDisplay, state, false, tagID, slot);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(DriverStation.getAlliance().isPresent())
    {
      DriverStation.Alliance alliance = DriverStation.getAlliance().get();
      if (alliance == Alliance.Red)
      {
        isRed = true;
      }
      else
      {
        isRed = false;
      }
    }
    if(slot != 2){
      tagID = fieldMap.getBestReefTagID(localizer.getPose());
      smartAlignCommand = create(drivetrain, localizer, fieldMap, mapDisplay, elevator, lidar, aprilTagFinder, tagID, isRed, slot);
    }

    else{
      tagID = fieldMap.getBestSourceTagID(localizer.getPose(), isRed);
      smartAlignCommand = createSource(drivetrain, localizer, fieldMap, mapDisplay, elevator, lidar, aprilTagFinder, tagID, slot);
    }
    
    smartAlignCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    smartAlignCommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
