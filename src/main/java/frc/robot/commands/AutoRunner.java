// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import choreo.trajectory.EventMarker;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.commands.Autos.Autos;
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.BallisticShot;
import frc.robot.subsystems.Bling;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.LaserCan;
import frc.robot.subsystems.Localizer;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.TargetFinder;
import frc.robot.subsystems.Turret;
import frc.robot.utilities.ShooterTable;

public class AutoRunner
{
  private final Drivetrain drivetrain;
  private final Localizer localizer;
  private final Kicker kicker;
  private final Climber climber;
  private final Flywheel flywheel;
  private final ShooterHood shooterHood;
  private final Spindexer spindexer;
  private final Turret turret;
  private final Collector collector;
  private final Intake intake;
  private final ShooterTable shooterTable;
  private final TargetFinder targetFinder;
  private final Bling bling;
  private final BallisticShot bs;

  public AutoRunner(
      Drivetrain drivetrain,
      AprilTagFinder tags,
      Localizer localizer,
      Kicker kicker,
      Climber climber,
      Flywheel flywheel,
      ShooterHood shooterHood,
      Spindexer spindexer,
      Turret turret,
      Collector collector,
      Intake intake,
      LaserCan laser,
      ShooterTable table,
      TargetFinder finder,
      Bling bling,
      BallisticShot bs)
  {
    this.drivetrain = drivetrain;
    this.localizer = localizer;
    this.kicker = kicker;
    this.climber = climber;
    this.flywheel = flywheel;
    this.shooterHood = shooterHood;
    this.spindexer = spindexer;
    this.turret = turret;
    this.collector = collector;
    this.intake = intake;
    this.targetFinder = finder;
    this.shooterTable = table;
    this.bling = bling;
    this.bs = bs;
  }

  // One-shot command helpers, matching the C++ subsystem-embedded RunOnce() factory methods.
  private Command intakeOut()
  {
    return Commands.runOnce(() -> intake.setPosition(Math.toRadians(-0.1)), intake);
  }

  private Command intakeIn()
  {
    return Commands.runOnce(() -> intake.setPosition(Math.toRadians(-122.0)), intake);
  }

  private Command collectSpeed(double metersPerSecond)
  {
    return Commands.runOnce(() -> collector.setVelocity(metersPerSecond), collector);
  }

  private Command spindexerSpeed(double metersPerSecond)
  {
    return Commands.runOnce(() -> spindexer.setVelocity(metersPerSecond), spindexer);
  }

  private Command kickerSpeed(double metersPerSecond)
  {
    return Commands.runOnce(() -> kicker.setVelocity(metersPerSecond), kicker);
  }

  private Command turretRotateToPos(double radians)
  {
    return Commands.runOnce(() -> turret.setPosition(radians), turret);
  }

  private Command climberPosition(double meters)
  {
    return Commands.runOnce(() -> climber.setPosition(meters), climber);
  }

  private Command climberHoldVelocity(double metersPerSecond)
  {
    return Commands.runOnce(() -> climber.setVelocity(metersPerSecond), climber);
  }

  private Command flywheelSpeed(double metersPerSecond)
  {
    return Commands.runOnce(() -> flywheel.setVelocity(metersPerSecond), flywheel);
  }

  private Command hoodPosition(double radians)
  {
    return Commands.runOnce(() -> shooterHood.setPosition(radians), shooterHood);
  }

  private Command trackHub()
  {
    return Autos.trackHub(turret, flywheel, shooterHood, targetFinder, shooterTable, bs);
  }

  private Command stopShooter()
  {
    return Commands.parallel(
        flywheelSpeed(0.0),
        spindexerSpeed(0.0),
        kickerSpeed(0.0),
        hoodPosition(ShooterHood.maxPositionRadians));
  }

  public Command eventParser(Optional<Trajectory<SwerveSample>> trajectory)
  {
    List<Command> autoRoutine = new ArrayList<>();

    if (trajectory.isPresent()) {
      Trajectory<SwerveSample> traj = trajectory.get();
      List<EventMarker> events = traj.events();

      double previousTime = 0.0;

      for (int e = 0; e < events.size(); e++) {
        EventMarker activeEvent = events.get(e);
        String eventType = activeEvent.event;

        double waitTime = activeEvent.timestamp - previousTime;
        autoRoutine.add(Commands.waitSeconds(waitTime));
        autoRoutine.add(new SmartDashPrint(eventType));

        previousTime = activeEvent.timestamp;

        switch (eventType) {
          case "StartSpindexer":
            autoRoutine.add(spindexerSpeed(Spindexer.shotSpeed));
            break;
          case "StartKicker":
            autoRoutine.add(kickerSpeed(Kicker.shotSpeed));
            break;
          case "StopSpindexer":
            autoRoutine.add(spindexerSpeed(0.0));
            break;
          case "StopKicker":
            autoRoutine.add(kickerSpeed(0.0));
            break;
          case "DeployIntake":
            autoRoutine.add(intakeOut());
            break;
          case "RetractIntake":
            autoRoutine.add(intakeIn());
            break;
          case "StartCollector":
            autoRoutine.add(collectSpeed(9.14)); // TODO: maybe multiplier should be higher
            break;
          case "StopCollector":
            autoRoutine.add(collectSpeed(0.0));
            break;
          case "ZeroTurret":
            autoRoutine.add(new ZeroTurret(turret));
            break;
          case "ZeroClimber":
            autoRoutine.add(new ZeroClimber(climber));
            break;
          case "BringUpClimber":
            autoRoutine.add(climberPosition(0.0582));
            break;
          case "BringDownClimber":
            autoRoutine.add(Commands.sequence(
                climberHoldVelocity(-0.67),
                Commands.waitSeconds(0.1),
                climberHoldVelocity(0.0)));
            break;
          case "TurretRotation125":
            autoRoutine.add(turretRotateToPos(Math.toRadians(125)));
            break;
          case "TurretRotation90":
            autoRoutine.add(turretRotateToPos(Math.toRadians(90)));
            break;
          case "TurretRotation70":
            autoRoutine.add(turretRotateToPos(Math.toRadians(70)));
            break;
          case "Shoot":
            autoRoutine.add(Commands.parallel(
                trackHub(),
                Commands.sequence(
                    Commands.waitSeconds(0.5),
                    spindexerSpeed(Spindexer.shotSpeed),
                    kickerSpeed(Kicker.shotSpeed),
                    Commands.waitSeconds(1.3),
                    intakeIn(),
                    Commands.waitSeconds(1.3),
                    intakeOut(),
                    Commands.waitSeconds(1.3),
                    intakeIn())).withTimeout(5.5));
            autoRoutine.add(Commands.parallel(intakeOut(), flywheelSpeed(0.0), spindexerSpeed(0.0), kickerSpeed(0.0), hoodPosition(ShooterHood.maxPositionRadians)));
            break;
          case "Shoot-Outpost":
            autoRoutine.add(Commands.parallel(
                trackHub(),
                Commands.sequence(
                    Commands.waitSeconds(1.0),
                    spindexerSpeed(Spindexer.shotSpeed),
                    kickerSpeed(Kicker.shotSpeed),
                    Commands.waitSeconds(6.0),
                    intakeIn(),
                    Commands.waitSeconds(1.0),
                    intakeOut(),
                    Commands.waitSeconds(2.0),
                    intakeIn())).withTimeout(15.0));
            break;
          case "Shoot-OutpostManual":
            autoRoutine.add(Commands.parallel(
                turretRotateToPos(Math.toRadians(-140)),
                flywheelSpeed(10.5),
                hoodPosition(0.267),
                Commands.sequence(
                    Commands.waitSeconds(1.0),
                    spindexerSpeed(Spindexer.shotSpeed),
                    kickerSpeed(Kicker.shotSpeed),
                    Commands.waitSeconds(6.0),
                    intakeIn(),
                    Commands.waitSeconds(1.0),
                    intakeOut(),
                    Commands.waitSeconds(2.0),
                    intakeIn())).withTimeout(15.0));
            break;
          case "CenterShoot":
            autoRoutine.add(Commands.parallel(
                trackHub(),
                Commands.sequence(
                    Commands.waitSeconds(0.5),
                    spindexerSpeed(Spindexer.shotSpeed),
                    kickerSpeed(Kicker.shotSpeed),
                    Commands.waitSeconds(1.3),
                    intakeIn(),
                    Commands.waitSeconds(1.3),
                    intakeOut(),
                    Commands.waitSeconds(1.3),
                    intakeIn(),
                    Commands.waitSeconds(1.0),
                    intakeOut())).withTimeout(6.75));
            autoRoutine.add(stopShooter());
            break;
          case "CenterShootOutpost":
            autoRoutine.add(Commands.parallel(
                trackHub(),
                Commands.sequence(
                    Commands.waitSeconds(0.85),
                    spindexerSpeed(6.5),
                    kickerSpeed(6.6),
                    Commands.waitSeconds(1.3),
                    intakeIn(),
                    Commands.waitSeconds(1.0),
                    intakeOut(),
                    Commands.waitSeconds(2.5),
                    intakeIn(),
                    Commands.waitSeconds(1.3),
                    intakeIn(),
                    Commands.waitSeconds(1.0),
                    intakeOut())).withTimeout(6.0)); // TODO: find the optimal timeout for the autos
            autoRoutine.add(stopShooter());
            break;
          case "CenterDepotToOutpostShoot":
            autoRoutine.add(Commands.parallel(
                trackHub(),
                Commands.sequence(
                    Commands.waitSeconds(0.7),
                    spindexerSpeed(Spindexer.shotSpeed),
                    kickerSpeed(Kicker.shotSpeed),
                    Commands.waitSeconds(0.5),
                    intakeIn(),
                    Commands.waitSeconds(0.5),
                    intakeOut(),
                    Commands.waitSeconds(0.5),
                    intakeIn(),
                    Commands.waitSeconds(0.5),
                    intakeIn(),
                    Commands.waitSeconds(0.5),
                    intakeOut(),
                    Commands.waitSeconds(0.5))).withTimeout(3.8)); // TODO: find the optimal timeout for the autos
            autoRoutine.add(stopShooter());
            break;
          case "Pause":
            autoRoutine.add(Commands.parallel(Commands.sequence()).withTimeout(0.5)); // TODO: find the optimal timeout for the autos
            break;
          case "CenterOutpostToClimbShoot":
            autoRoutine.add(Commands.parallel(
                trackHub(),
                Commands.sequence(
                    Commands.waitSeconds(1.0),
                    spindexerSpeed(Spindexer.shotSpeed),
                    kickerSpeed(Kicker.shotSpeed),
                    Commands.waitSeconds(1.5),
                    intakeIn(),
                    Commands.waitSeconds(0.5),
                    intakeOut(),
                    Commands.waitSeconds(0.5),
                    collectSpeed(0.0),
                    intakeIn(),
                    Commands.waitSeconds(0.5),
                    intakeOut(),
                    Commands.waitSeconds(0.5),
                    intakeIn(),
                    Commands.waitSeconds(1.0),
                    intakeOut(),
                    Commands.waitSeconds(1.0),
                    intakeIn(),
                    Commands.waitSeconds(1.0),
                    intakeOut(),
                    Commands.waitSeconds(1.0),
                    intakeIn())).withTimeout(7.8));
            autoRoutine.add(Commands.parallel(flywheelSpeed(0.0), spindexerSpeed(0.0), kickerSpeed(0.0), hoodPosition(ShooterHood.maxPositionRadians), intakeIn()));
            break;
          case "ShootMovingOutpost":
            autoRoutine.add(Commands.parallel(
                trackHub(),
                Commands.sequence(
                    Commands.waitSeconds(1.3),
                    spindexerSpeed(6.5),
                    kickerSpeed(6.6),
                    Commands.waitSeconds(0.5),
                    Commands.runOnce(bling::blingPurple, bling), // purple: about to go in
                    Commands.waitSeconds(0.5),
                    intakeIn(),
                    Commands.runOnce(bling::blingWhite, bling),
                    Commands.waitSeconds(1.0),
                    intakeOut(),
                    Commands.waitSeconds(3.5),
                    Commands.runOnce(bling::blingPurple, bling),
                    Commands.waitSeconds(0.5),
                    intakeIn(),
                    Commands.runOnce(bling::blingWhite, bling),
                    Commands.waitSeconds(2.0),
                    intakeOut(),
                    Commands.waitSeconds(1.5),
                    Commands.runOnce(bling::blingPurple, bling),
                    Commands.waitSeconds(0.5),
                    intakeIn(),
                    Commands.runOnce(bling::blingWhite, bling))).withTimeout(15.0));
            break;
          case "ShootDoublePath":
            autoRoutine.add(Commands.parallel(
                trackHub(),
                Commands.sequence(
                    Commands.waitSeconds(1.3),
                    spindexerSpeed(Spindexer.shotSpeed),
                    kickerSpeed(Kicker.shotSpeed))).withTimeout(5.0));
            autoRoutine.add(stopShooter());
            break;
          case "ShootBumpAuto":
            autoRoutine.add(Commands.parallel(
                trackHub(),
                Commands.sequence(
                    collectSpeed(0.0),
                    Commands.waitSeconds(1.2),
                    spindexerSpeed(Spindexer.shotSpeed),
                    kickerSpeed(Kicker.shotSpeed),
                    Commands.waitSeconds(1.0),
                    intakeIn(),
                    Commands.waitSeconds(0.5),
                    intakeOut(),
                    Commands.waitSeconds(0.5),
                    intakeIn(),
                    Commands.waitSeconds(0.5),
                    intakeOut(),
                    Commands.waitSeconds(0.5),
                    intakeIn(),
                    Commands.waitSeconds(0.5),
                    intakeOut(),
                    Commands.waitSeconds(0.5),
                    intakeIn(),
                    Commands.waitSeconds(0.5),
                    intakeOut(),
                    Commands.waitSeconds(0.5),
                    intakeIn(),
                    Commands.waitSeconds(0.5),
                    intakeOut(),
                    Commands.waitSeconds(0.5),
                    intakeIn(),
                    Commands.waitSeconds(0.5),
                    intakeOut(),
                    Commands.waitSeconds(0.5),
                    intakeIn())).withTimeout(10.0));
            autoRoutine.add(stopShooter());
            break;
          case "ShootBump":
            autoRoutine.add(Commands.parallel(
                trackHub(),
                Commands.sequence(
                    Commands.waitSeconds(1.2),
                    spindexerSpeed(Spindexer.shotSpeed),
                    kickerSpeed(Kicker.shotSpeed),
                    Commands.waitSeconds(1.2),
                    intakeIn(),
                    Commands.waitSeconds(0.6),
                    intakeOut(),
                    Commands.waitSeconds(0.6),
                    intakeIn(),
                    Commands.waitSeconds(0.6),
                    intakeOut(),
                    Commands.waitSeconds(0.6),
                    intakeIn(),
                    Commands.waitSeconds(0.6),
                    intakeOut(),
                    Commands.waitSeconds(0.6),
                    intakeIn())).withTimeout(5.4));
            autoRoutine.add(Commands.parallel(intakeOut(), flywheelSpeed(0.0), spindexerSpeed(0.0), kickerSpeed(0.0), hoodPosition(ShooterHood.maxPositionRadians), collectSpeed(9.14)));
            break;
          case "ShootFollow":
            autoRoutine.add(Commands.parallel(
                trackHub(),
                Commands.sequence(
                    Commands.waitSeconds(0.5),
                    climberPosition(0.0582),
                    spindexerSpeed(Spindexer.shotSpeed),
                    kickerSpeed(Kicker.shotSpeed),
                    Commands.waitSeconds(1.0),
                    intakeIn(),
                    Commands.waitSeconds(0.5),
                    intakeOut(),
                    Commands.waitSeconds(0.5),
                    intakeIn())).withTimeout(5.0));
            autoRoutine.add(stopShooter());
            break;
          case "ShootLeftFollow":
            autoRoutine.add(Commands.parallel(
                trackHub(),
                Commands.sequence(
                    Commands.waitSeconds(0.5),
                    spindexerSpeed(Spindexer.shotSpeed),
                    kickerSpeed(Kicker.shotSpeed),
                    Commands.waitSeconds(1.0),
                    intakeIn(),
                    Commands.waitSeconds(0.5),
                    intakeOut(),
                    Commands.waitSeconds(0.5),
                    intakeIn(),
                    Commands.waitSeconds(0.5),
                    intakeOut())).withTimeout(5.2));
            autoRoutine.add(stopShooter());
            break;
          case "ShootLeftClimb":
            autoRoutine.add(Commands.parallel(
                trackHub(),
                Commands.sequence(
                    Commands.waitSeconds(0.5),
                    climberPosition(0.0582),
                    spindexerSpeed(Spindexer.shotSpeed),
                    kickerSpeed(Kicker.shotSpeed),
                    Commands.waitSeconds(1.0),
                    intakeIn(),
                    Commands.waitSeconds(0.5),
                    climberPosition(0.0582),
                    intakeOut(),
                    Commands.waitSeconds(0.5),
                    intakeIn())).withTimeout(2.5));
            autoRoutine.add(Commands.parallel(flywheelSpeed(0.0), spindexerSpeed(0.0), kickerSpeed(0.0), hoodPosition(ShooterHood.maxPositionRadians), intakeIn()));
            break;
          default:
            break;
        }
      }

      return Commands.sequence(autoRoutine.toArray(new Command[0]));
    } else {
      System.err.println("Error: returning Idle Command");
      return Commands.idle();
    }
  }

  public Command partGenerator(Optional<Trajectory<SwerveSample>> trajectory, double delay)
  {
    List<Command> parts = new ArrayList<>();

    if (delay > 0.0) {
      parts.add(Commands.waitSeconds(delay)); // If we have a delay add it to the 1st part.
    }

    if (trajectory.isPresent()) {
      Trajectory<SwerveSample> traj = trajectory.get();

      for (int s = 0; s < traj.splits().size(); s++) {
        Optional<Trajectory<SwerveSample>> splitTraj = traj.getSplit(s);

        Command part = Commands.parallel(
            new DrivePath(drivetrain, localizer, splitTraj),
            eventParser(splitTraj));

        parts.add(part);
      }

      return Commands.sequence(parts.toArray(new Command[0]));
    } else {
      System.err.println("Auto Runner Part Generator not have a trajectory");
      // Matches C++: constructs a SmartDashPrint here but never schedules it, so it never actually runs.
      return Commands.idle(); // You have to return something!?
    }
  }

  public Command prep(double delay)
  {
    return Commands.parallel(
        Commands.waitSeconds(delay + 0.01),
        new ScheduleCommand(new ZeroTurret(turret, true)), // Unsafe command
        new ScheduleCommand(new ZeroClimber(climber, true)), // Unsafe command
        new IntakeOut(intake, true)).withTimeout(5.0); // Absolute maximum time...
  }

  public Command prepWithoutIntake(double delay)
  {
    return Commands.parallel(
        Commands.waitSeconds(delay + 0.01),
        new ScheduleCommand(new ZeroTurret(turret, true)), // Unsafe command
        new ScheduleCommand(new ZeroClimber(climber, true))).withTimeout(5.0); // Absolute maximum time...
  }

  public Command create(Optional<Trajectory<SwerveSample>> trajectory, double startDelay, boolean putIntakeOut)
  {
    Command partsList = partGenerator(trajectory, startDelay);

    return Commands.sequence(
        putIntakeOut ? prep(startDelay) : prepWithoutIntake(startDelay),
        partsList).withTimeout(30.0); // Absolute maximum time... real auto is 20s.
  }
}
