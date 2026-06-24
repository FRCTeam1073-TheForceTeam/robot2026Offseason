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
import frc.robot.commands.AlgaeAutoEject;
import frc.robot.commands.AlgaeAutoGrab;
import frc.robot.commands.AlgaeEject;
import frc.robot.commands.AlgaeGrab;
import frc.robot.commands.AlgaeHold;
import frc.robot.commands.AlgaeOpen;
import frc.robot.commands.AlgaePivotTeleop;
import frc.robot.commands.AlignToTag;
import frc.robot.commands.AlignToTagRelative;
import frc.robot.commands.CancelLoadCoral;
import frc.robot.commands.CANdleObserver;
import frc.robot.commands.ClimberTeleop;
import frc.robot.commands.CoralElevatorTeleop;
import frc.robot.commands.CoralElevatorToHeight;
import frc.robot.commands.CoralEndeffectorTeleop;
import frc.robot.commands.DisengageClimber;
import frc.robot.commands.EngageClimber;
import frc.robot.commands.FloorAlgaeCollect;
import frc.robot.commands.FloorLoadCoral;
import frc.robot.commands.FloorPickupCollectTeleop;
import frc.robot.commands.FloorPickupPivotTeleop;
import frc.robot.commands.FloorScoreCoral;
import frc.robot.commands.LidarAlign;
import frc.robot.commands.LoadCoral;
import frc.robot.commands.ProcessorScore;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.SetFloorPivotPos;
import frc.robot.commands.SmartAlign;
import frc.robot.commands.StowElevator;
import frc.robot.commands.StowSequence;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.ZeroAlgaePivot;
import frc.robot.commands.ZeroClimber;
import frc.robot.commands.ZeroElevator;
import frc.robot.commands.ZeroFloorPivotPos;
import frc.robot.commands.Autos.AutoCenterStart;
import frc.robot.commands.Autos.AutoLeftStart;
import frc.robot.commands.Autos.AutoRightStart;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.commands.Autos.TestAuto;
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandStates;
import frc.robot.subsystems.CoralElevator;
import frc.robot.subsystems.CoralEndeffector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FieldMap;
import frc.robot.subsystems.FloorPickupCollect;
import frc.robot.subsystems.FloorPickupPivot;
import frc.robot.subsystems.Lidar;
import frc.robot.subsystems.Localizer;
import frc.robot.subsystems.MapDisplay;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.CANdleControl;

public class RobotContainer implements Consumer<String> // need the interface for onChange
{
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final OI m_OI = new OI();
  private final AprilTagFinder m_aprilTagFinder = new AprilTagFinder();
  private final Field2d m_field = new Field2d();
  private final FieldMap m_fieldMap = new FieldMap();
  private final Localizer m_localizer = new Localizer(m_drivetrain, m_fieldMap, m_aprilTagFinder);
  private final MapDisplay m_MapDisplay = new MapDisplay(m_drivetrain, m_localizer, m_fieldMap);
  private final CoralElevator m_coralElevator = new CoralElevator();
  private final Climber m_climber = new Climber();
  private final Lidar m_lidar = new Lidar();
  //private final Lidar m_lidar = null; // Disabled temporarily.
  private final CANdleControl m_CANdleControl = new CANdleControl();
  private final CoralEndeffector m_coralEndeffector = new CoralEndeffector();
  private final AlgaePivot m_algaePivot = new AlgaePivot();
  private final CommandStates m_commandStates = new CommandStates();
  private final FloorPickupPivot m_floorPickupPivot = new FloorPickupPivot();
  private final FloorPickupCollect m_floorPickupCollect = new FloorPickupCollect();


  private final ZeroElevator cmd_zeroElevator = new ZeroElevator(m_coralElevator);
  private final CoralElevatorTeleop cmd_coralElevatorTeleop = new CoralElevatorTeleop(m_coralElevator, m_OI);
  private final CoralEndeffectorTeleop cmd_coralEndeffectorTeleop = new CoralEndeffectorTeleop(m_coralEndeffector, m_OI);
  private final LoadCoral cmd_loadCoral = new LoadCoral(m_coralEndeffector);
  private final ScoreCoral cmd_scoreCoral = new ScoreCoral(m_coralEndeffector);
  private final CoralElevatorToHeight cmd_coralElevatorToL2 = new CoralElevatorToHeight(m_coralElevator, 2, false);
  private final CoralElevatorToHeight cmd_coralElevatorToL3 = new CoralElevatorToHeight(m_coralElevator, 3, false);
  private final CoralElevatorToHeight cmd_troughRaiseElevator = new CoralElevatorToHeight(m_coralElevator, 1, false);
  private final CoralElevatorToHeight cmd_coralElevatorToL4 = new CoralElevatorToHeight(m_coralElevator, 4, false);
  private final CoralElevatorToHeight cmd_coralElevatorToHighA = new CoralElevatorToHeight(m_coralElevator, 6, false);
  private final CoralElevatorToHeight cmd_coralElevatorToBarge = new CoralElevatorToHeight(m_coralElevator, 7, false);
  private final CancelLoadCoral cmd_cancelLoadCoral = new CancelLoadCoral(m_coralEndeffector);
  private final AlignToTag cmd_alignToTag = new AlignToTag(m_drivetrain, m_localizer, m_fieldMap, m_MapDisplay, m_commandStates, true, 0, -1);
  private final ClimberTeleop cmd_climberTeleop = new ClimberTeleop(m_climber, m_OI);
  private final ZeroClimber cmd_zeroClimber = new ZeroClimber(m_climber);
  private final EngageClimber cmd_engageClimber = new EngageClimber(m_climber);
  private final DisengageClimber cmd_disengageClimber = new DisengageClimber(m_climber);
  private final CANdleObserver cmd_candleObserver = new CANdleObserver(m_CANdleControl, m_coralEndeffector, m_climber, m_OI, m_commandStates, m_drivetrain);
  private final LidarAlign cmd_lidarAlign = new LidarAlign(m_lidar, m_drivetrain, m_commandStates);
  private final AlignToTagRelative cmd_localAlign = new AlignToTagRelative(m_drivetrain, m_aprilTagFinder, m_commandStates, 0, 0);
  private final StowElevator cmd_stowElevator = new StowElevator(m_coralElevator);
  private final AlgaeGrab cmd_algaeGrab = new AlgaeGrab(m_coralEndeffector, false);
  private final AlgaeEject cmd_algaeEject = new AlgaeEject(m_coralEndeffector, m_algaePivot);
  private final AlgaePivotTeleop cmd_algaePivotTeleop = new AlgaePivotTeleop(m_OI, m_algaePivot);
  private final ZeroAlgaePivot cmd_zeroAlgaePivot = new ZeroAlgaePivot(m_algaePivot);
  private final TeleopDrive cmd_teleopDrive = new TeleopDrive(m_drivetrain, m_OI, m_aprilTagFinder, m_localizer, m_lidar);
  private final SmartAlign cmd_smartAlignReefLeft = new SmartAlign(m_drivetrain, m_localizer, m_commandStates, m_fieldMap, m_MapDisplay, m_coralElevator, m_lidar, m_aprilTagFinder, -1);
  private final SmartAlign cmd_smartAlignReefRight = new SmartAlign(m_drivetrain, m_localizer, m_commandStates, m_fieldMap, m_MapDisplay, m_coralElevator, m_lidar, m_aprilTagFinder, 1);
  private final SmartAlign cmd_smartAlignSource = new SmartAlign(m_drivetrain, m_localizer, m_commandStates, m_fieldMap, m_MapDisplay, m_coralElevator, m_lidar, m_aprilTagFinder, 2);
  private final Command cmd_algaeAutoGrab = AlgaeAutoGrab.create(m_algaePivot, m_coralEndeffector);
  private final Command cmd_algaeAutoEject = AlgaeAutoEject.create(m_coralEndeffector, m_algaePivot);
  private final Command cmd_algaeOpen = AlgaeOpen.create(m_algaePivot,m_coralEndeffector, m_OI);
  private final Command cmd_algaeHold = AlgaeHold.create(m_algaePivot, m_coralEndeffector);
  private final SmartAlign cmd_smartAlignReefCenter = new SmartAlign(m_drivetrain, m_localizer, m_commandStates, m_fieldMap, m_MapDisplay, m_coralElevator, m_lidar, m_aprilTagFinder, 0);
  private final FloorPickupCollectTeleop cmd_floorPickupCollectTeleop = new FloorPickupCollectTeleop(m_floorPickupCollect);
  private final FloorPickupPivotTeleop cmd_floorPickupPivotTeleop = new FloorPickupPivotTeleop(m_floorPickupPivot, m_OI);
  private final ZeroFloorPivotPos cmd_zeroFloorPivotPos = new ZeroFloorPivotPos(m_floorPickupPivot, m_floorPickupCollect);
  private final FloorScoreCoral cmd_floorScoreCoral = new FloorScoreCoral(m_floorPickupCollect, m_floorPickupPivot);
  private final FloorLoadCoral cmd_floorLoadCoral = new FloorLoadCoral(m_floorPickupPivot, m_floorPickupCollect);
  private final FloorAlgaeCollect cmd_floorAlgaeCollect = new FloorAlgaeCollect(m_floorPickupCollect, m_floorPickupPivot);
  private final ProcessorScore cmd_processorScore = new ProcessorScore(m_floorPickupCollect, m_floorPickupPivot);
  private final StowSequence cmd_stowSequence = new StowSequence(m_coralElevator);


  private boolean isRed;
  private int level;
  private boolean isRainbow = false;

  public boolean haveInitStartPos = false;

  private final SendableChooser<String> m_positionChooser = new SendableChooser<>();
  private static final String noPosition = "No Position";
  private static final String rightPosition = "Right Auto";
  private static final String leftPosition = "Left Auto";
  private static final String centerPosition = "Center Auto";
  private static final String centerPositionX = "Center Auto X";
  private static final String testAuto = "Test Auto";
  
  private final SendableChooser<String> m_levelChooser = new SendableChooser<>();
  private static final String noLevelAuto = "No Level";
  private static final String leave = "Leave";
  private static final String scoreL1 = "Score L1";
  private static final String scoreL2 = "Score L2";
  private static final String scoreL3 = "Score L3";
  private static final String scoreL4 = "Score L4";
  private static final String score2L4 = "Score 2 L4";
  private static final String score3L4 = "Score 3 L4";
  private static final String algaeGrab = "Algae Grab";
  private static final String bargeScore = "Barge Score";
  
  private static final String zeroClawAndLift = "Zero Claw And Lift";

  public RobotContainer() 
  {
    CommandScheduler.getInstance().setDefaultCommand(m_drivetrain, cmd_teleopDrive);
    CommandScheduler.getInstance().setDefaultCommand(m_coralElevator, cmd_coralElevatorTeleop);
    CommandScheduler.getInstance().setDefaultCommand(m_coralEndeffector, cmd_coralEndeffectorTeleop);
    CommandScheduler.getInstance().setDefaultCommand(m_climber, cmd_climberTeleop);
    CommandScheduler.getInstance().setDefaultCommand(m_CANdleControl, cmd_candleObserver);
    CommandScheduler.getInstance().setDefaultCommand(m_algaePivot, cmd_algaePivotTeleop);
    CommandScheduler.getInstance().setDefaultCommand(m_floorPickupPivot, cmd_floorPickupPivotTeleop);
    CommandScheduler.getInstance().setDefaultCommand(m_floorPickupCollect, cmd_floorPickupCollectTeleop);

    SmartDashboard.putData(m_drivetrain);
    SmartDashboard.putData(m_OI);
    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putData(m_localizer);

    m_positionChooser.setDefaultOption("No Position", noPosition);
    m_positionChooser.addOption("Right Position", rightPosition);
    m_positionChooser.addOption("Left Position", leftPosition);
    m_positionChooser.addOption("Center Position", centerPosition);
    m_positionChooser.addOption("Center Position X", centerPositionX);
    m_positionChooser.addOption("Zero Claw and Lift", zeroClawAndLift);
    m_positionChooser.addOption("Test Auto", testAuto);

    m_levelChooser.setDefaultOption("No Level", noLevelAuto);
    m_levelChooser.addOption("Leave", leave);
    m_levelChooser.addOption("Score L1", scoreL1);
    m_levelChooser.addOption("Score L2", scoreL2);
    m_levelChooser.addOption("Score L3", scoreL3);
    m_levelChooser.addOption("Score L4", scoreL4);
    m_levelChooser.addOption("Score 2 L4", score2L4);
    m_levelChooser.addOption("Score 3 L4", score3L4);
    m_levelChooser.addOption("Algae Grab", algaeGrab);
    m_levelChooser.addOption("Barge Score", bargeScore);

    SmartDashboard.putData("Position Chooser", m_positionChooser);
    SmartDashboard.putData("Level Chooser", m_levelChooser);

    m_positionChooser.onChange(this::accept); // this is so we can reset the start position

    configureBindings();
  }

  private void configureBindings() {
    Trigger disengageClimber = new Trigger(m_OI::getOperatorDisengageClimber);
      disengageClimber.onTrue(cmd_disengageClimber);

    Trigger engageClimber = new Trigger(m_OI::getOperatorEngageClimber);
      engageClimber.onTrue(cmd_engageClimber);

    Trigger zeroClimber = new Trigger(m_OI::getOperatorZeroClimber);
      zeroClimber.onTrue(cmd_zeroClimber);

    Trigger loadFloorCoral = new Trigger(m_OI::getOperatorFirstPlayer);
      loadFloorCoral.whileTrue(cmd_floorLoadCoral);

    Trigger scoreCoral = new Trigger(m_OI::getOperatorScoralCoral);
      scoreCoral.onTrue(cmd_scoreCoral);
      
    Trigger elevatorL2 = new Trigger(m_OI::getOperatorL2);
      elevatorL2.whileTrue(cmd_coralElevatorToL2);

    Trigger elevatorL3 = new Trigger(m_OI::getOperatorL3);
      elevatorL3.whileTrue(cmd_coralElevatorToL3);
    
    Trigger elevatorL4 = new Trigger(m_OI::getOperatorL4);
      elevatorL4.whileTrue(cmd_coralElevatorToL4);

    Trigger troughScore = new Trigger(m_OI::getOperatorL1);
      troughScore.whileTrue(cmd_troughRaiseElevator);
    
    // Trigger alignToTag = new Trigger(m_OI::getDriverAlignButtons);
    //   alignToTag.whileTrue(cmd_alignToTag);

    // Trigger lidarAlign = new Trigger(m_OI::getDriverBButton);
    //   lidarAlign.whileTrue(cmd_lidarAlign);

    Trigger tagCenterAlign = new Trigger(m_OI::getDriverAButton);
      tagCenterAlign.whileTrue(cmd_smartAlignReefCenter);

    Trigger sourceAlign = new Trigger(m_OI::getDriverBButton);
      sourceAlign.whileTrue(cmd_smartAlignSource);

    Trigger tagLeftAlign = new Trigger(m_OI::getDriverXButton);
      tagLeftAlign.whileTrue(cmd_smartAlignReefLeft);
    
    Trigger tagRightAlign = new Trigger(m_OI::getDriverYButton);
      tagRightAlign.whileTrue(cmd_smartAlignReefRight);

    Trigger localAlign = new Trigger(m_OI::getDriverMenuButton);
      localAlign.whileTrue(cmd_localAlign);
    
    // Trigger zeroElevator = new Trigger(m_OI::getOperatorZeroElevator);
    //   zeroElevator.onTrue(cmd_zeroElevator);

    Trigger zeroElevator = new Trigger(m_OI::getOperatorZeroElevator);
    zeroElevator.onTrue(cmd_stowSequence.create());

    Trigger elevatorBarge = new Trigger(m_OI::getOperatorBargeScoreButton);
      elevatorBarge.whileTrue(cmd_coralElevatorToBarge);
    
    // Trigger elevatorHighAlgae = new Trigger(m_OI::getOperatorHighAlgae);
    //   elevatorHighAlgae.whileTrue(cmd_coralElevatorToHighA);
    
    Trigger floorScoreCoral = new Trigger(m_OI::getOperatorTwoPlayerButton);
      floorScoreCoral.whileTrue(cmd_floorScoreCoral);

    Trigger algaeOpen = new Trigger(m_OI::getOperatorAlgaeOpen);
    algaeOpen.onTrue(cmd_algaeOpen);

    Trigger algaeZero = new Trigger(m_OI::getOperatorAlgaeZero);
    algaeZero.onTrue(cmd_zeroAlgaePivot);

    Trigger ejectAlgaeAuto = new Trigger(m_OI::getOperatorAlgaeEject);
      ejectAlgaeAuto.onTrue(cmd_algaeAutoEject);
    
    Trigger floorLoadAlgae = new Trigger(m_OI::getOperatorFloorLoadAlgae);
      floorLoadAlgae.whileTrue(cmd_floorAlgaeCollect);
    
    Trigger floorScoreAlgae = new Trigger(m_OI::getOperatorFloorAlgaeScore);
      floorScoreAlgae.whileTrue(cmd_processorScore);
    
    Trigger zeroFloorMech = new Trigger(m_OI::getOperatorFloorMechUp);
      zeroFloorMech.whileTrue(cmd_zeroFloorPivotPos);
    
  } 

  public void autonomousInit()
  {
    m_CANdleControl.clearAnim();
    // m_floorPickupPivot.setRotatorPos(0);
  }

    public Command getAutonomousCommand() 
  {
   // return Commands.print("No autonomous command configured");
   // -1 to indicate no auto select
    switch(m_levelChooser.getSelected())
    {
      case noLevelAuto:
        level = -1;
        break;
      case leave:
        level = 0;
        break;
      case scoreL1:
        level = 1;
        break;
      case scoreL2:
        level = 2;
        break;
      case scoreL3:
        level = 3;
        break;
      case scoreL4:
        level = 4;                  
        break;
      case score2L4:
        level = 5;
        break;
      case score3L4:
        level = 11;
        break;
      case algaeGrab:
        level = 6;
        break;
      case bargeScore:
        level = 7;
        break;
      default:
        level = -1;
        break;
    }


    switch(m_positionChooser.getSelected())
    {
      // case zeroClawAndLift:
      //   return ZeroClawAndLift.create(m_climberClaw, m_climberLift);
      case noPosition:
        return new WaitCommand(20.0);
      case leftPosition:
        return AutoLeftStart.create(level, isRed, m_drivetrain, m_localizer, m_fieldMap, m_climber, m_coralEndeffector, m_coralElevator, m_lidar, m_aprilTagFinder, m_commandStates);
      case rightPosition:
        return AutoRightStart.create(level, isRed, m_drivetrain, m_localizer, m_fieldMap, m_climber, m_coralEndeffector, m_coralElevator, m_lidar, m_aprilTagFinder, m_commandStates);
      case centerPosition:
        return AutoCenterStart.create(level, isRed, m_drivetrain, m_localizer, m_fieldMap, m_climber, m_coralEndeffector, m_coralElevator, m_lidar, m_aprilTagFinder, m_algaePivot, m_commandStates, false);
      case centerPositionX:
        return AutoCenterStart.create(level, isRed, m_drivetrain, m_localizer, m_fieldMap, m_climber, m_coralEndeffector, m_coralElevator, m_lidar, m_aprilTagFinder, m_algaePivot, m_commandStates, true);
      case testAuto:
        return TestAuto.create(m_drivetrain, m_localizer, m_fieldMap);
      default:
        return null;
    }
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
    
    m_CANdleControl.clearAnim();
    haveInitStartPos = false;
  }

  public boolean findStartPos() 
  {
      if(!DriverStation.getAlliance().isPresent() || m_positionChooser.getSelected().equals("No Position")) {
        return false;
      }

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
      }
      return false;
  }

  public boolean disabledPeriodic() 
  {
    if(DriverStation.getAlliance().isPresent())
    {
      int totalLED = m_CANdleControl.getTotalLED();
      int candleNum = m_CANdleControl.getCandleNum();

      DriverStation.Alliance alliance = DriverStation.getAlliance().get();
      if(alliance == Alliance.Blue) {
        m_CANdleControl.setRGB(0, 0, 255, candleNum, totalLED);
      }
      else if (alliance == Alliance.Red){
        m_CANdleControl.setRGB(255, 0, 0, candleNum, totalLED);
      }
      else{
        m_CANdleControl.setRGB(255, 255, 255, candleNum, totalLED);
      }
  }
    
    return findStartPos();
  }

  @Override
  public void accept(String t) // gets called every time the selected position changes so the start position is reinitialized
  {
    haveInitStartPos = false;  
  }
}
