// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoRunner;
import frc.robot.commands.ClimberTeleop;
import frc.robot.commands.CollectorTeleop;
import frc.robot.commands.FlywheelTeleop;
import frc.robot.commands.HoodTeleop;
import frc.robot.commands.IntakeTeleop;
import frc.robot.commands.DumperBlockerTeleop;
import frc.robot.commands.KickerTeleop;
import frc.robot.commands.SpindexerTeleop;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.TestFlywheel;
import frc.robot.commands.TestHood;
import frc.robot.commands.TurretTeleop;
import frc.robot.commands.ZeroClimber;
import frc.robot.commands.ZeroHood;
import frc.robot.commands.ZeroIntake;
import frc.robot.commands.ZeroDumperBlocker;
import frc.robot.commands.ZeroTurret;
import frc.robot.commands.Autos.Autos;
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.BallisticShot;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FieldMap;
import frc.robot.subsystems.FieldMapDisplay;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.DumperBlocker;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Localizer;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.TargetFinder;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.ZoneFinder;
import frc.robot.utilities.AutoLog;
import frc.robot.utilities.DashboardNames;
import frc.robot.utilities.ShooterTable;
import frc.robot.utilities.TrajectoryCache;
import frc.robot.utilities.TrajectoryWarmup;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    public static final String noLevelAuto = "No Auto";
    public static final String startLine = "Start Line";
    public static final String centerHub = "CenterHub";
    public static final String centerDepotOutpost = "CenterDepotOutpost";
    public static final String centerDepotClimb = "CenterDepotClimb";
    public static final String rightBumpSteal = "RightBumpSteal";
    public static final String rightTrenchHalfOutpost = "RightTrenchHalfOutpost";
    public static final String rightTrenchHalfDouble = "RightTrenchHalfDouble";
    public static final String rightTrenchHalfDoubleBump = "RightTrenchHalfDoubleBump";
    public static final String leftTrenchHalfDouble = "LeftTrenchHalfDouble";
    public static final String leftTrenchHalfDoubleBump = "LeftTrenchHalfDoubleBump";
    public static final String leftBumpFull = "LeftBumpFull";
    public static final String basicTest = "BasicTest";
    public static final String basicAuto = "BasicAuto";
    public static final String rightBumpFollow = "RightBumpFollow";
    public static final String leftBumpFollow = "LeftBumpFollow";

    private final SendableChooser<String> levelChooser = new SendableChooser<>();

    private boolean haveTraj = false;
    private String autoTraj = "none";

    // Trajectories are loaded on a background thread. Only the selected auto is loaded, when it is selected.
    private final TrajectoryCache trajectoryCache = new TrajectoryCache(name -> Choreo.loadTrajectory(name));
    private final ExecutorService preloader = Executors.newSingleThreadExecutor(r -> {
                Thread thread = new Thread(r, "auto-preload");
                thread.setDaemon(true);
                return thread;
    });
    private volatile boolean classesReady = false;
    // Command construction must stay on the main thread, so it is done once per selection in disabledPeriodic.
    private String commandWarmedAuto = "";
    private String preloadRequest = "";

    // Create these subsystems first!
    private final OI oi = new OI();
    private final Drivetrain drivetrain = new Drivetrain();

    // Must be here because localizer depends on this due to moving camera.
    private final Turret turret = new Turret();

    private final FieldMap fieldMap = new FieldMap();
    private final AprilTagFinder tagFinder = new AprilTagFinder(turret);
    private final Localizer localizer = new Localizer(drivetrain, tagFinder);
    private final FieldMapDisplay fieldDisplay = new FieldMapDisplay(drivetrain, localizer, fieldMap);
    private final ZoneFinder zoneFinder = new ZoneFinder(localizer);
    private final TargetFinder targetFinder = new TargetFinder(localizer, zoneFinder);
    private final BallisticShot ballisticShot = new BallisticShot(targetFinder);

    private final Climber climber = new Climber();

    private final ShooterTable shooterTable = new ShooterTable();
    private final Intake intake = new Intake();
    private final DumperBlocker dumperBlocker = new DumperBlocker();
    private final Collector collector = new Collector();
    private final Spindexer spindexer = new Spindexer();
    private final Kicker kicker = new Kicker();
    private final ShooterHood shooterHood = new ShooterHood();
    private final Flywheel flywheel = new Flywheel();
    // LaserCan laser; -- not instantiated, matching C++ (commented out there too).

    private final AutoRunner autoRunner = new AutoRunner(drivetrain, tagFinder, localizer, kicker, climber, flywheel, shooterHood,
            spindexer, turret, collector, intake, null, shooterTable, targetFinder, ballisticShot);

    // Just used for launching test commands, separate from OI and other controls.
    private final CommandXboxController operatorController = new CommandXboxController(1);

    private boolean controlBindings = false;

    private final double startDelaySeconds = 0.0;

    public RobotContainer()
    {
        System.err.println("\tDrivetrain created...");

        targetFinder.setBallisticShot(ballisticShot);

        System.err.println("\tShooter table created...");
        System.err.println("\tIntake created...");
        System.err.println("\tDumper Blocker created...");
        System.err.println("\tCollector created...");
        System.err.println("\tSpindexer created...");
        System.err.println("\tKicker created...");
        System.err.println("\tShooterHood created...");
        System.err.println("\tFlywheel created...");
        System.err.println("\tBling created...");

        System.err.println("Mechanisms created...");

        // Default commands are assigned in teleopInit(), after all subsystems are created, to avoid
        // using uninitialized subsystems in default commands.

        System.err.println("\tDefault commands assigned...");

        // Autonomous Chooser:
        levelChooser.setDefaultOption("No Level", noLevelAuto);
        levelChooser.addOption("Start_Line", startLine);
        levelChooser.addOption("Center_Hub", centerHub);
        levelChooser.addOption("Center_Depot_Outpost", centerDepotOutpost);
        levelChooser.addOption("Center_Depot_Climb", centerDepotClimb);
        levelChooser.addOption("Right_Bump_Steal", rightBumpSteal);
        levelChooser.addOption("Right_Trench_Half_Outpost", rightTrenchHalfOutpost);
        levelChooser.addOption("Right_Trench_Half_Double", rightTrenchHalfDouble);
        levelChooser.addOption("Right_Trench_Half_Double_Bump", rightTrenchHalfDoubleBump);
        levelChooser.addOption("Left_Trench_Half_Double", leftTrenchHalfDouble);
        levelChooser.addOption("Left_Trench_Half_Double_Bump", leftTrenchHalfDoubleBump);
        levelChooser.addOption("Outliers_Right", leftBumpFull);
        levelChooser.addOption("Basic Test", basicTest);
        levelChooser.addOption("Basic Auto", basicAuto);
        levelChooser.addOption("Right_Bump_Follow", rightBumpFollow);
        levelChooser.addOption("Left_Bump_Follow", leftBumpFollow);

        SmartDashboard.putData(DashboardNames.AUTO_LEVEL_CHOOSER.getKey(), levelChooser);

        SmartDashboard.putNumber(DashboardNames.AUTO_START_DELAY.getKey(), startDelaySeconds);

        // Configure the button bindings
        configureBindings();
        System.err.println("Controller bindings configured...");

        preloadAutos();
    }

    public Command getAutonomousCommand()
    {
        try {
            // Grab our delay in seconds:
            double delay = SmartDashboard.getNumber(DashboardNames.AUTO_START_DELAY.getKey(), 0.0);
            System.err.println("**Auto Start Delay(s): " + delay);

            String selected = levelChooser.getSelected();
            AutoLog.event("selected '" + selected + "', start delay " + delay + " s, prewarmed " + isPrewarmed()
                    + " (classes " + classesReady + ")");

            if (selected.equals(startLine)) {
                return Commands.sequence(Commands.waitSeconds(delay), new ZeroTurret(turret), new ZeroClimber(climber),
                        Autos.basicAutoShot(spindexer, kicker, turret, flywheel, shooterHood, targetFinder, shooterTable, ballisticShot));
            } else if (selected.equals(centerHub)) {
                return Commands.sequence(Commands.waitSeconds(delay), new ZeroTurret(turret), new ZeroClimber(climber),
                        Autos.hubAuto(spindexer, kicker, turret, flywheel, shooterHood));
            } else if (AutoCatalog.runsRoutine(selected)) {
                Optional<String> trajFile = AutoCatalog.trajectoryFile(selected);
                if (trajFile.isPresent()) {
                    AutoLog.event("trajectory " + trajFile.get() + " preloaded: " + trajectoryCache.isReady(trajFile.get()));
                }

                long trajStart = System.nanoTime();
                Optional<Trajectory<SwerveSample>> trajectory = trajFile.map(trajectoryCache::get).orElse(Optional.empty());
                AutoLog.event(String.format("trajectory ready in %.1f ms", (System.nanoTime() - trajStart) / 1e6));

                if (trajFile.isPresent() && trajectory.isEmpty()) {
                    // Refuse to run a routine without its path rather than drive the wrong one.
                    System.err.println("AUTO MISSING TRAJECTORY: " + trajFile.get() + ", not running auto");
                    AutoLog.event("AUTO MISSING TRAJECTORY, not running auto");
                    return Commands.idle();
                }

                boolean putIntakeOut = true;

                if (selected.equals(centerDepotOutpost)) {
                    putIntakeOut = false;
                } else if (selected.equals(leftBumpFull)) {
                    putIntakeOut = false;
                } else if (selected.equals(centerDepotClimb)) {
                    putIntakeOut = false;
                } else if (selected.equals(rightBumpSteal)) {
                    putIntakeOut = false;
                } else if (selected.equals(rightTrenchHalfDoubleBump)) {
                    putIntakeOut = false;
                } else if (selected.equals(leftTrenchHalfDouble)) {
                    putIntakeOut = false;
                } else if (selected.equals(leftTrenchHalfDoubleBump)) {
                    putIntakeOut = false;
                } else if (selected.equals(rightTrenchHalfDouble)) {
                    putIntakeOut = false;
                } else if (selected.equals(basicTest)) {
                    putIntakeOut = false;
                }

                SmartDashboard.putBoolean(DashboardNames.AUTO_PUT_INTAKE_OUT.getKey(), putIntakeOut);
                SmartDashboard.putNumber(DashboardNames.AUTO_START_TIME.getKey(), edu.wpi.first.wpilibj.Timer.getFPGATimestamp());

                long buildStart = System.nanoTime();
                Command routine = autoRunner.create(trajectory, delay, putIntakeOut);
                AutoLog.event(String.format("auto command built in %.1f ms", (System.nanoTime() - buildStart) / 1e6));
                return routine;
            }
        } catch (RuntimeException e) {
            System.err.println("Get Autonomous Command Threw Exception");
            return Commands.idle(); // A do-nothing command from the commands factory.
        }

        return Commands.idle(); // Do nothing.
    }

    // Called from Robot
    public void disabledInit()
    {
        haveTraj = false;
    }

    /** Called every disabled loop. Returns true if the selected auto can run as soon as it is enabled. */
    public boolean disabledPeriodic()
    {
        String selected = levelChooser.getSelected();
        autoTraj = selected;

        if (!selected.equals(preloadRequest)) {
            preloadRequest = selected;
            AutoCatalog.trajectoryFile(selected).ifPresent(this::requestPreload);
        }

        // Cheap, non-blocking checks only. Loading happens on the preload thread.
        prewarmCommandStep(selected);

        return AutoCatalog.trajectoryFile(selected).map(trajectoryCache::isReady).orElse(true);
    }

    public boolean isPrewarmed()
    {
        return classesReady && commandWarmedAuto.equals(levelChooser.getSelected());
    }

    /** Loads the auto classes on the preload thread. Class loading is slow enough to overrun the main loop. */
    private void preloadAutos()
    {
        preloader.submit(() -> {
                    long start = System.nanoTime();
                    TrajectoryWarmup.loadClasses();
                    classesReady = true;
                    System.err.println("Auto preload classes loaded in " + (System.nanoTime() - start) / 1e6 + " ms");
        });
    }

    /** Loads only the selected auto's trajectory, on the preload thread, so enable does not have to. */
    private void requestPreload(String name)
    {
        if (trajectoryCache.isReady(name)) {
            return;
        }
        preloader.submit(() -> {
                    long start = System.nanoTime();
                    trajectoryCache.get(name).ifPresent(TrajectoryWarmup::touchSamples);
                    System.err.println("Auto preload " + name + " in " + (System.nanoTime() - start) / 1e6 + " ms");
        });
    }

    /**
     * Builds the selected auto's command once, discarding it without scheduling, so its classes are
     * loaded before enable. Waits until the selected trajectory is loaded. Done on the main thread
     * because command construction is not thread-safe.
     */
    private void prewarmCommandStep(String selected)
    {
        if (!classesReady || commandWarmedAuto.equals(selected)) {
            return;
        }

        if (!AutoCatalog.runsRoutine(selected)) {
            commandWarmedAuto = selected;
            return;
        }

        Optional<String> trajFile = AutoCatalog.trajectoryFile(selected);
        if (trajFile.isPresent() && !trajectoryCache.isReady(trajFile.get())) {
            return;
        }

        Optional<Trajectory<SwerveSample>> trajectory = trajFile.map(trajectoryCache::get).orElse(Optional.empty());
        autoRunner.create(trajectory, startDelaySeconds, true);
        commandWarmedAuto = selected;
        System.err.println("Auto command warmup finished for " + selected);
    }

    // Called from Robot
    public void teleopInit()
    {
        // Default commands are assigned here after all subsystems are created, to avoid using
        // uninitialized subsystems in default commands.
        drivetrain.setDefaultCommand(new TeleopDrive(drivetrain, oi, localizer));
        intake.setDefaultCommand(new IntakeTeleop(intake, oi, zoneFinder));
        dumperBlocker.setDefaultCommand(new DumperBlockerTeleop(dumperBlocker, intake, oi));
        collector.setDefaultCommand(new CollectorTeleop(collector, oi, drivetrain));
        spindexer.setDefaultCommand(new SpindexerTeleop(spindexer, kicker, oi));
        kicker.setDefaultCommand(new KickerTeleop(kicker, oi));
        shooterHood.setDefaultCommand(new HoodTeleop(shooterHood, oi, targetFinder, shooterTable, zoneFinder, ballisticShot));
        flywheel.setDefaultCommand(new FlywheelTeleop(flywheel, oi, targetFinder, shooterTable, ballisticShot));
        turret.setDefaultCommand(new TurretTeleop(turret, oi, targetFinder, drivetrain));
        climber.setDefaultCommand(new ClimberTeleop(climber, oi, zoneFinder));

        // If the turret has not yet seen zero, zero it now.
        if (!turret.hasZero()) {
            CommandScheduler.getInstance().schedule(new ZeroTurret(turret));
        }

        // TODO: Consider moving this back to configureBindings().
        // Moved here to de-conflict DPAD in test mode.
        if (!controlBindings) {
            operatorController.povLeft().onTrue(new ZeroIntake(intake));
            operatorController.povUp().onTrue(new ZeroTurret(turret));
            operatorController.povRight().onTrue(new ZeroHood(shooterHood));
            operatorController.povDown().onTrue(new ZeroClimber(climber));

            new edu.wpi.first.wpilibj2.command.button.Trigger(() -> {
                        boolean aPressed = oi.getOperatorAButton();
                        if (aPressed) {
                            System.out.println("DEBUG: Operator A Button Pressed");
                }
                        return aPressed;
            })
                .onTrue(new ZeroDumperBlocker(dumperBlocker));

            controlBindings = true;
        }

        // Parking brake: toggle with driver B button
        //   new edu.wpi.first.wpilibj2.command.button.Trigger(() -> {
        //       boolean bPressed = oi.getDriverBButton();
        //       if (bPressed) {
        //           System.err.println("DEBUG: Driver B button pressed! Current brake state: " + drivetrain.getParkingBrake());
        //       }
        //       return bPressed;
        //   })
        //       .onTrue(edu.wpi.first.wpilibj2.command.Commands.runOnce(() -> {
        //           System.err.println("DEBUG: Executing parking brake toggle");
        //           drivetrain.parkingBrake(!drivetrain.getParkingBrake());
        //           System.err.println("DEBUG: Parking brake toggled to: " + drivetrain.getParkingBrake());
        //       }, drivetrain));

        //   controlBindings = true;
        // }
    }

    private void configureBindings()
    {
        // Command bindings moved to teleopInit().
    }

    // Called from Robot
    public void testInit()
    {
        System.err.println("***** TestInit ****");

        // In test mode we run these manually:
        flywheel.removeDefaultCommand();
        shooterHood.removeDefaultCommand();

        // Launch some commands for test mode:
        CommandScheduler.getInstance().schedule(new TestFlywheel(flywheel, oi));
        CommandScheduler.getInstance().schedule(new TestHood(shooterHood, oi));
    }

    public void setHubActive(boolean active)
    {
        oi.setHubActive(active);
    }

    public boolean getHaveTraj()
    {
        return haveTraj;
    }

    public void setHaveTraj(boolean haveTraj)
    {
        this.haveTraj = haveTraj;
    }

    public String getAutoTraj()
    {
        return autoTraj;
    }

    public String getSelectedAuto()
    {
        return levelChooser.getSelected();
    }
}
