// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Localizer;
import frc.robot.subsystems.OI;

public class TeleopDrive extends Command
{
  public static final double JOYSTICK_DEADZONE = 0.15;

  private static final double maximumLinearVelocity = 4.5;
  private static final double maximumRotationVelocity = 2.0 * Math.PI;

  private final Drivetrain drivetrain;
  private final OI oi;
  private final Localizer localizer;

  private ChassisSpeeds speeds;

  private boolean fieldCentric;
  private boolean lastFieldCentricButton;
  private boolean slowMode;
  private boolean lastYPressed;
  private boolean lastXPressed;
  private boolean fastRotation;

  private double avgTorque;
  private double currentTime;

  // Used to flip the sign based on which alliance we are set for:
  private int allianceSign;

  public TeleopDrive(Drivetrain drivetrain, OI oi, Localizer localizer)
  {
    this.drivetrain = drivetrain;
    this.oi = oi;
    this.localizer = localizer;

    allianceSign = 0;
    fieldCentric = true;
    lastFieldCentricButton = true;
    slowMode = false;
    lastYPressed = false;
    lastXPressed = false;
    fastRotation = true;

    addRequirements(drivetrain);
  }

  public TeleopDrive(Drivetrain drivetrain, OI oi)
  {
    this(drivetrain, oi, null);
  }

  @Override
  public void initialize()
  {
    System.err.println("TeleopDrive Init");
    setAlliance(); // Set the alliance sign.
  }

  @Override
  public void execute()
  {
    if (allianceSign == 0) {
      setAlliance(); // Try to figure it out if not already set.
    }

    double leftY = oi.getDriverLeftY();
    double leftX = oi.getDriverLeftX();
    double rightX = oi.getDriverRightX();
    avgTorque = drivetrain.getAverageLoad();
    currentTime = Timer.getMatchTime();

    if (oi.getDriverLeftBumper() && lastFieldCentricButton == false) {
      fieldCentric = !fieldCentric;
    }
    lastFieldCentricButton = oi.getDriverLeftBumper();

    boolean driverDPadUp = oi.getDriverDPadUp();
    boolean driverDPadDown = oi.getDriverDPadDown();
    boolean driverDPadLeft = oi.getDriverDPadLeft();
    boolean driverDPadRight = oi.getDriverDPadRight();
    int driverDPadAngle = oi.getDriverDPadAngle();

    // set deadzones
    if (Math.abs(leftY) < JOYSTICK_DEADZONE) {
      leftY = 0.0;
    }
    if (Math.abs(leftX) < JOYSTICK_DEADZONE) {
      leftX = 0.0;
    }
    if (Math.abs(rightX) < JOYSTICK_DEADZONE) {
      rightX = 0.0;
    }

    double vx = MathUtil.clamp(allianceSign * sign(leftY + 0.01)
        * (maximumLinearVelocity / (maximumLinearVelocity - 1))
        * (Math.pow(maximumLinearVelocity, Math.abs(leftY)) - 1), -maximumLinearVelocity, maximumLinearVelocity);
    double vy = MathUtil.clamp(allianceSign * sign(leftX + 0.01)
        * (maximumLinearVelocity / (maximumLinearVelocity - 1))
        * (Math.pow(maximumLinearVelocity, Math.abs(leftX)) - 1), -maximumLinearVelocity, maximumLinearVelocity);
    double omega = MathUtil.clamp(sign(rightX + 0.01)
        * (maximumRotationVelocity / (maximumRotationVelocity - 1))
        * (Math.pow(maximumRotationVelocity, Math.abs(rightX)) - 1), -maximumRotationVelocity, maximumRotationVelocity);

    if (!lastYPressed && oi.getDriverYButton()) {
      slowMode = !slowMode;
    }
    lastYPressed = oi.getDriverYButton();
    if (slowMode) {
      vx *= 0.4;
      vy *= 0.4;
    }

    if (!lastXPressed && oi.getDriverXButton()) {
      fastRotation = !fastRotation;
    }
    lastXPressed = oi.getDriverXButton();

    if (!fastRotation) {
      omega *= 0.4;
    }

    SmartDashboard.putBoolean("TeleopDrive/Slow Mode", slowMode);
    SmartDashboard.putNumber("TeleopDrive/vx", vx);
    SmartDashboard.putNumber("TeleopDrive/vy", vy);
    SmartDashboard.putNumber("TeleopDrive/omega", omega);
    SmartDashboard.putNumber("TeleopDrive/AvgTorque", avgTorque);
    SmartDashboard.putBoolean("TeleopDrive/FieldCentric", fieldCentric);
    SmartDashboard.putNumber("TeleopDrive/leftX", leftX);
    SmartDashboard.putNumber("TeleopDrive/leftY", leftY);
    SmartDashboard.putNumber("TeleopDrive/rightX", rightX);
    SmartDashboard.putNumber("TeleopDrive/Driver DPad angle", driverDPadAngle);
    SmartDashboard.putBoolean("TeleopDrive/Driver DPad Up", driverDPadUp);
    SmartDashboard.putBoolean("TeleopDrive/Driver DPad Down", driverDPadDown);
    SmartDashboard.putBoolean("TeleopDrive/Driver DPad Left", driverDPadLeft);
    SmartDashboard.putBoolean("TeleopDrive/Driver DPad Right", driverDPadRight);
    SmartDashboard.putNumber("TeleopDrive/Maximum Rotation Velocity", maximumRotationVelocity);
    SmartDashboard.putBoolean("TeleopDrive/Fast Rotation", fastRotation);

    // odometry centric drive
    if (fieldCentric) {
      Rotation2d rotation;
      if (localizer != null) {
        rotation = localizer.getPose().getRotation();
      } else {
        rotation = drivetrain.getGyroHeading();
      }

      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, rotation);
      drivetrain.setTargetChassisSpeeds(speeds);
    } else { // robot centric drive
      drivetrain.setTargetChassisSpeeds(new ChassisSpeeds(allianceSign * -vx, allianceSign * -vy, omega));
    }
  }

  @Override
  public void end(boolean interrupted)
  {
    if (interrupted) {
      System.err.println("TeleopDrive: Interrupted!");
    }
  }

  @Override
  public boolean isFinished()
  {
    return false;
  }

  private void setAlliance()
  {
    var alliance = DriverStation.getAlliance();

    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Red) {
        allianceSign = 1;
        System.err.println("TeleopDrive:: RED Alliance");
      } else {
        allianceSign = -1;
        System.err.println("TeleopDrive:: BLUE Alliance");
      }
    } else {
      System.err.println("WARNING: TeleopDrive:: Alliance not set.");
    }
  }

  private static double sign(double value)
  {
    return value / Math.abs(value);
  }
}
