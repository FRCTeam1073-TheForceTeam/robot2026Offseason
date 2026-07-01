// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.Autos.TestAuto;
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FieldMap;
import frc.robot.subsystems.FieldMapDisplay;
import frc.robot.subsystems.Localizer;
import frc.robot.subsystems.OI;

public class RobotContainer implements Consumer<String> // need the interface for onChange
{
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final OI m_OI = new OI();
  // private final AprilTagFinder m_aprilTagFinder = new AprilTagFinder();
  // private final Field2d m_field = new Field2d();
  // private final FieldMap m_fieldMap = new FieldMap();
  // private final Localizer m_localizer = new Localizer(m_drivetrain, m_fieldMap, m_aprilTagFinder);
  // private final MapDisplay m_MapDisplay = new MapDisplay(m_drivetrain, m_localizer, m_fieldMap);
  //private final Lidar m_lidar = null; // Disabled temporarily.

  // private final TeleopDrive cmd_teleopDrive = new TeleopDrive(m_drivetrain, m_OI, m_aprilTagFinder);
  


  private boolean isRed;
  private int level;
  private boolean isRainbow = false;

  public boolean haveInitStartPos = false;

  private final SendableChooser<String> m_positionChooser = new SendableChooser<>();
  private static final String testAuto = "Test Auto";

  public RobotContainer() 
  {
    // CommandScheduler.getInstance().setDefaultCommand(m_drivetrain, cmd_teleopDrive);

    SmartDashboard.putData(m_drivetrain);
    SmartDashboard.putData(m_OI);
    // SmartDashboard.putData("Field", m_field);
    // SmartDashboard.putData(m_localizer);

    m_positionChooser.setDefaultOption("Test Auto", testAuto);

    SmartDashboard.putData("Position Chooser", m_positionChooser);

    m_positionChooser.onChange(this::accept); // this is so we can reset the start position

    configureBindings();
  }

  private void configureBindings() {
    // Trigger disengageClimber = new Trigger(m_OI::getOperatorDisengageClimber);
    //   disengageClimber.onTrue(cmd_disengageClimber);

    // Trigger engageClimber = new Trigger(m_OI::getOperatorEngageClimber);
    //   engageClimber.onTrue(cmd_engageClimber);

    // Trigger zeroClimber = new Trigger(m_OI::getOperatorZeroClimber);
    //   zeroClimber.onTrue(cmd_zeroClimber);

    // Trigger loadFloorCoral = new Trigger(m_OI::getOperatorFirstPlayer);
    //   loadFloorCoral.whileTrue(cmd_floorLoadCoral);

    // Trigger scoreCoral = new Trigger(m_OI::getOperatorScoralCoral);
    //   scoreCoral.onTrue(cmd_scoreCoral);
      
    // Trigger elevatorL2 = new Trigger(m_OI::getOperatorL2);
    //   elevatorL2.whileTrue(cmd_coralElevatorToL2);

    // Trigger elevatorL3 = new Trigger(m_OI::getOperatorL3);
    //   elevatorL3.whileTrue(cmd_coralElevatorToL3);
    
    // Trigger elevatorL4 = new Trigger(m_OI::getOperatorL4);
    //   elevatorL4.whileTrue(cmd_coralElevatorToL4);

    // Trigger troughScore = new Trigger(m_OI::getOperatorL1);
    //   troughScore.whileTrue(cmd_troughRaiseElevator);
    
    // // Trigger alignToTag = new Trigger(m_OI::getDriverAlignButtons);
    // //   alignToTag.whileTrue(cmd_alignToTag);

    // // Trigger lidarAlign = new Trigger(m_OI::getDriverBButton);
    // //   lidarAlign.whileTrue(cmd_lidarAlign);

    // Trigger tagCenterAlign = new Trigger(m_OI::getDriverAButton);
    //   tagCenterAlign.whileTrue(cmd_smartAlignReefCenter);

    // Trigger sourceAlign = new Trigger(m_OI::getDriverBButton);
    //   sourceAlign.whileTrue(cmd_smartAlignSource);

    // Trigger tagLeftAlign = new Trigger(m_OI::getDriverXButton);
    //   tagLeftAlign.whileTrue(cmd_smartAlignReefLeft);
    
    // Trigger tagRightAlign = new Trigger(m_OI::getDriverYButton);
    //   tagRightAlign.whileTrue(cmd_smartAlignReefRight);

    // Trigger localAlign = new Trigger(m_OI::getDriverMenuButton);
    //   localAlign.whileTrue(cmd_localAlign);
    
    // // Trigger zeroElevator = new Trigger(m_OI::getOperatorZeroElevator);
    // //   zeroElevator.onTrue(cmd_zeroElevator);

    // Trigger zeroElevator = new Trigger(m_OI::getOperatorZeroElevator);
    // zeroElevator.onTrue(cmd_stowSequence.create());

    // Trigger elevatorBarge = new Trigger(m_OI::getOperatorBargeScoreButton);
    //   elevatorBarge.whileTrue(cmd_coralElevatorToBarge);
    
    // // Trigger elevatorHighAlgae = new Trigger(m_OI::getOperatorHighAlgae);
    // //   elevatorHighAlgae.whileTrue(cmd_coralElevatorToHighA);
    
    // Trigger floorScoreCoral = new Trigger(m_OI::getOperatorTwoPlayerButton);
    //   floorScoreCoral.whileTrue(cmd_floorScoreCoral);

    // Trigger algaeOpen = new Trigger(m_OI::getOperatorAlgaeOpen);
    // algaeOpen.onTrue(cmd_algaeOpen);

    // Trigger algaeZero = new Trigger(m_OI::getOperatorAlgaeZero);
    // algaeZero.onTrue(cmd_zeroAlgaePivot);

    // Trigger ejectAlgaeAuto = new Trigger(m_OI::getOperatorAlgaeEject);
    //   ejectAlgaeAuto.onTrue(cmd_algaeAutoEject);
    
    // Trigger floorLoadAlgae = new Trigger(m_OI::getOperatorFloorLoadAlgae);
    //   floorLoadAlgae.whileTrue(cmd_floorAlgaeCollect);
    
    // Trigger floorScoreAlgae = new Trigger(m_OI::getOperatorFloorAlgaeScore);
    //   floorScoreAlgae.whileTrue(cmd_processorScore);
    
    // Trigger zeroFloorMech = new Trigger(m_OI::getOperatorFloorMechUp);
    //   zeroFloorMech.whileTrue(cmd_zeroFloorPivotPos);
    
  } 

  public void autonomousInit()
  {
    // m_CANdleControl.clearAnim();
    // m_floorPickupPivot.setRotatorPos(0);
  }

    public Command getAutonomousCommand() 
  {
   // return Commands.print("No autonomous command configured");
   // -1 to indicate no auto select


    // switch(m_positionChooser.getSelected())
    // {
    //   // case zeroClawAndLift:
    //   //   return ZeroClawAndLift.create(m_climberClaw, m_climberLift);
    //   case testAuto:
    //     return TestAuto.create(m_drivetrain, m_localizer, m_fieldMap);
    //   default:
    //     return null;
    // }
    return null;
  }

  public void printAllFalseDiagnostics()
  {
    boolean isDisabled = DriverStation.isDisabled();
    boolean allOK = true;
    // Set allOK to the results of the printDiagnostics method for each subsystem, separated by &&
    allOK = true;
    //TODO: Add each subsystem
    SmartDashboard.putBoolean("Engine light", allOK);
  }

  public Command getTeleopCommand()
  {
    return null;
  }

  public Command getDisabledCommand() 
  {
    return null;
  }

  public void disabledInit() 
  {
    
    // m_CANdleControl.clearAnim();
    haveInitStartPos = false;
  }

  public boolean findStartPos() 
  {
      if(!DriverStation.getAlliance().isPresent() || m_positionChooser.getSelected().equals("No Position")) {
        return false;
      }
      /*
      //create a bool for pose is set
      double centerY = 4.026;
      int allianceSign = 1;
      String initPosition = m_positionChooser.getSelected();
      
      double centerX = 8.774; // this is the default 
      double startLineOffset = 12.227 -8.774 - 2.24; //id 10 x value - center x value - offset from reef to startline
      Pose2d startPos = new Pose2d();
      SmartDashboard.putString("Alliance", "None");
  
      if(DriverStation.getAlliance().isPresent())
      {
        DriverStation.Alliance alliance = DriverStation.getAlliance().get();
        if(alliance == Alliance.Blue) {
          allianceSign = -1;
        }
        if (initPosition.equals(leftPosition)) {
          centerY -= allianceSign * 2.013;
        }
        else if(initPosition.equals(rightPosition)) {
          centerY += allianceSign * 2.013;
        }
  
        if (alliance == Alliance.Blue)
       {
          isRed = false;
          SmartDashboard.putString("Alliance", "Blue");
          // startPos = new Pose2d(centerX - startLineOffset, centerY, new Rotation2d(Math.PI)); //startline
          startPos = new Pose2d(centerX - startLineOffset, centerY, new Rotation2d(Math.PI)); //startline
        }
        else if (alliance == Alliance.Red)
        { 
          isRed = true;
          SmartDashboard.putString("Alliance", "Red");
          startPos = new Pose2d(centerX + startLineOffset, centerY, new Rotation2d(0)); //startline
        }
        else
        {
          return false;
          // SmartDashboard.putString("Alliance", "Null");
          // isRed = false;
          // startPos = new Pose2d(0, 0, new Rotation2d(0));
        }
        m_drivetrain.resetOdometry(startPos);
        m_localizer.resetPose(startPos);
        SmartDashboard.putNumber("RobotContainer/Start Pose X", startPos.getX());
        SmartDashboard.putNumber("RobotContainer/Start Pose Y", startPos.getY());
        SmartDashboard.putNumber("RobotContainer/Start Pose Rotation", startPos.getRotation().getRadians());
        SmartDashboard.putBoolean("Alliance", isRed); //True = Red, False = Blue ***NEED TO EDIT ON ELASTIC
        return true;
      }*/
      return false;
  }

  public boolean disabledPeriodic() 
  {
    if(DriverStation.getAlliance().isPresent())
    {
      // int totalLED = m_CANdleControl.getTotalLED();
      // int candleNum = m_CANdleControl.getCandleNum();

      DriverStation.Alliance alliance = DriverStation.getAlliance().get();
      // if(alliance == Alliance.Blue) {
      //   m_CANdleControl.setRGB(0, 0, 255, candleNum, totalLED);
      // }
      // else if (alliance == Alliance.Red){
      //   m_CANdleControl.setRGB(255, 0, 0, candleNum, totalLED);
      // }
      // else{
      //   m_CANdleControl.setRGB(255, 255, 255, candleNum, totalLED);
      // }
  }
    
    return findStartPos();
  }

  @Override
  public void accept(String t) // gets called every time the selected position changes so the start position is reinitialized
  {
    haveInitStartPos = false;  
  }
}
