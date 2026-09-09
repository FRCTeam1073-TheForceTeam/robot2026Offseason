// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.DashboardNames;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class OI extends SubsystemBase
{
    @AutoLog
    public static class OIInputs
    {
        public double driverLeftX = 0.0;
        public double driverLeftY = 0.0;
        public double driverRightX = 0.0;
        public double driverRightY = 0.0;
        public double driverLeftTrigger = 0.0;
        public double driverRightTrigger = 0.0;
        public boolean driverAButton = false;
        public boolean driverBButton = false;
        public boolean driverXButton = false;
        public boolean driverYButton = false;
        public boolean driverMenuButton = false;
        public boolean driverViewButton = false;
        public boolean driverLeftBumper = false;
        public boolean driverRightBumper = false;
        public int driverDPadAngle = -1;
        public double operatorLeftX = 0.0;
        public double operatorLeftY = 0.0;
        public double operatorRightX = 0.0;
        public double operatorRightY = 0.0;
        public double operatorLeftTrigger = 0.0;
        public double operatorRightTrigger = 0.0;
        public boolean operatorAButton = false;
        public boolean operatorBButton = false;
        public boolean operatorXButton = false;
        public boolean operatorYButton = false;
        public boolean operatorMenuButton = false;
        public boolean operatorViewButton = false;
        public boolean operatorLeftBumper = false;
        public boolean operatorRightBumper = false;
        public int operatorDPadAngle = -1;
        public boolean ballisticShotMode = false;
    }

    @AutoLog
    public static class DriverStationInputs
    {
        public boolean allianceKnown = false;
        public String alliance = "Unknown";
        public String gameSpecificMessage = "";
        public boolean isAutonomous = false;
        public boolean isTeleop = false;
        public boolean isEnabled = false;
        public double matchTime = 0.0;
    }

    private final XboxController driverController;
    private final XboxController operatorController;
    private final OIInputsAutoLogged inputs = new OIInputsAutoLogged();
    private final DriverStationInputsAutoLogged dsInputs = new DriverStationInputsAutoLogged();

    // TODO: make debouncers for individual buttons.
    private final Debouncer debouncer = new Debouncer(0.05, Debouncer.DebounceType.kBoth);

    private double leftXZero;
    private double leftYZero;
    private double rightXZero;
    private double rightYZero;

    private boolean hubActive = false;
    private boolean lastHubActive = false;

    // Top-level control of ballistic shot mode:
    private boolean ballisticShot = true;
    private boolean lastOperatorAButton = false;

    public OI()
    {
        setName("OI");
        driverController = new XboxController(0);
        operatorController = new XboxController(1);
    }

    @Override
    public void periodic()
    {
        // Populate OI inputs (joysticks + buttons)
        inputs.driverLeftX = driverController.getLeftX();
        inputs.driverLeftY = driverController.getLeftY();
        inputs.driverRightX = -1 * driverController.getRightX();
        inputs.driverRightY = driverController.getRightY();
        inputs.driverLeftTrigger = driverController.getLeftTriggerAxis();
        inputs.driverRightTrigger = driverController.getRightTriggerAxis();
        inputs.driverAButton = driverController.getAButton();
        inputs.driverBButton = driverController.getBButton();
        inputs.driverXButton = driverController.getXButton();
        inputs.driverYButton = driverController.getYButton();
        inputs.driverMenuButton = driverController.getStartButton();
        inputs.driverViewButton = driverController.getBackButton();
        inputs.driverLeftBumper = driverController.getLeftBumperButton();
        inputs.driverRightBumper = driverController.getRightBumperButton();
        inputs.driverDPadAngle = driverController.getPOV();
        inputs.operatorLeftX = -1 * operatorController.getLeftX();
        inputs.operatorLeftY = operatorController.getLeftY();
        inputs.operatorRightX = operatorController.getRightX();
        inputs.operatorRightY = operatorController.getRightY();
        inputs.operatorLeftTrigger = operatorController.getLeftTriggerAxis();
        inputs.operatorRightTrigger = operatorController.getRightTriggerAxis();
        inputs.operatorAButton = operatorController.getAButton();
        inputs.operatorBButton = operatorController.getBButton();
        inputs.operatorXButton = operatorController.getXButton();
        inputs.operatorYButton = operatorController.getYButton();
        inputs.operatorMenuButton = operatorController.getStartButton();
        inputs.operatorViewButton = operatorController.getBackButton();
        inputs.operatorLeftBumper = operatorController.getLeftBumperButton();
        inputs.operatorRightBumper = operatorController.getRightBumperButton();
        inputs.operatorDPadAngle = operatorController.getPOV();
        inputs.ballisticShotMode = ballisticShot;
        Logger.processInputs("OI", inputs);

        // Populate DriverStation inputs
        var alliance = DriverStation.getAlliance();
        dsInputs.allianceKnown = alliance.isPresent();
        dsInputs.alliance = alliance.isPresent() ? alliance.get().name() : "Unknown";
        dsInputs.gameSpecificMessage = DriverStation.getGameSpecificMessage();
        dsInputs.isAutonomous = DriverStation.isAutonomousEnabled();
        dsInputs.isTeleop = DriverStation.isTeleopEnabled();
        dsInputs.isEnabled = DriverStation.isEnabled();
        dsInputs.matchTime = DriverStation.getMatchTime();
        Logger.processInputs("DriverStation", dsInputs);

        boolean aButton = operatorController.getAButton();
        if (aButton && !lastOperatorAButton) {
            ballisticShot = !ballisticShot;
        }
        lastOperatorAButton = aButton;

        SmartDashboard.putBoolean(DashboardNames.OI_BALLISTIC_SHOT.getKey(), ballisticShot);
    }

    public boolean ballisticShotMode()
    {
        return ballisticShot;
    }

    public double getDriverLeftX()
    {
        return driverController.getLeftX();
    }

    public double getDriverLeftY()
    {
        return driverController.getLeftY();
    }

    public double getDriverRightX()
    {
        return -1 * driverController.getRightX();
    }

    public double getDriverRightY()
    {
        return driverController.getRightY();
    }

    public double getOperatorLeftX()
    {
        return -1 * operatorController.getLeftX();
    }

    public double getOperatorLeftY()
    {
        return operatorController.getLeftY();
    }

    public double getOperatorRightX()
    {
        return operatorController.getRightX();
    }

    public double getOperatorRightY()
    {
        return operatorController.getRightY();
    }

    public double getDriverLeftTrigger()
    {
        return driverController.getLeftTriggerAxis();
    }

    public double getDriverRightTrigger()
    {
        return driverController.getRightTriggerAxis();
    }

    public double getOperatorLeftTrigger()
    {
        return operatorController.getLeftTriggerAxis();
    }

    public double getOperatorRightTrigger()
    {
        return operatorController.getRightTriggerAxis();
    }

    public boolean getDriverAButton()
    {
        return driverController.getAButton();
    }

    public boolean getDriverBButton()
    {
        return driverController.getBButton();
    }

    public boolean getDriverXButton()
    {
        return driverController.getXButton();
    }

    public boolean getDriverYButton()
    {
        return driverController.getYButton();
    }

    public boolean getDriverMenuButton()
    {
        return driverController.getStartButton();
    }

    public boolean getDriverViewButton()
    {
        return driverController.getBackButton();
    }

    public boolean getDriverLeftBumper()
    {
        return driverController.getLeftBumperButton();
    }

    public boolean getDriverRightBumper()
    {
        return driverController.getRightBumperButton();
    }

    public int getDriverDPadAngle()
    {
        return driverController.getPOV();
    }

    public boolean getDriverDPadUp()
    {
        return driverController.getPOV() == 0;
    }

    public boolean getDriverDPadRight()
    {
        return driverController.getPOV() == 90;
    }

    public boolean getDriverDPadLeft()
    {
        return driverController.getPOV() == 270;
    }

    public boolean getDriverDPadDown()
    {
        return driverController.getPOV() == 180;
    }

    public void driverRumble()
    {
        driverController.setRumble(RumbleType.kBothRumble, 1.0);
    }

    public void driverStopRumble()
    {
        driverController.setRumble(RumbleType.kBothRumble, 0.0);
    }

    public boolean driverLeftStickPress()
    {
        return driverController.getLeftStickButton();
    }

    public boolean getOperatorAButton()
    {
        return operatorController.getAButton();
    }

    public boolean getOperatorBButton()
    {
        return operatorController.getBButton();
    }

    public boolean getOperatorXButton()
    {
        return operatorController.getXButton();
    }

    public boolean getOperatorYButton()
    {
        return operatorController.getYButton();
    }

    public boolean getOperatorMenuButton()
    {
        return operatorController.getStartButton();
    }

    public boolean getOperatorViewButton()
    {
        return operatorController.getBackButton();
    }

    public boolean getOperatorLeftBumper()
    {
        return operatorController.getLeftBumperButton();
    }

    public boolean getOperatorRightBumper()
    {
        return operatorController.getRightBumperButton();
    }

    public int getOperatorDPadAngle()
    {
        return operatorController.getPOV();
    }

    public boolean getOperatorDPadUp()
    {
        return operatorController.getPOV() == 0;
    }

    public boolean getOperatorDPadRight()
    {
        return operatorController.getPOV() == 90;
    }

    public boolean getOperatorDPadLeft()
    {
        return operatorController.getPOV() == 270;
    }

    public boolean getOperatorDPadDown()
    {
        return operatorController.getPOV() == 180;
    }

    public void operatorRumble()
    {
        operatorController.setRumble(RumbleType.kBothRumble, 1.0);
    }

    public void operatorStopRumble()
    {
        operatorController.setRumble(RumbleType.kBothRumble, 0.0);
    }

    public void zeroDriverController()
    {
        leftXZero = getDriverLeftX();
        leftYZero = getDriverLeftY();
        rightXZero = getDriverRightX();
        rightYZero = getDriverRightY();
    }

    public void zeroOperatorController()
    {
        leftXZero = getOperatorLeftX();
        leftYZero = getOperatorLeftY();
        rightXZero = getOperatorRightX();
        rightYZero = getOperatorRightY();
    }

    public void setHubActive(boolean active)
    {
        hubActive = active;
    }
}
