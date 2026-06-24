// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// Free drive controller buttons: press left joystick, press right joystick
// Free operator controller buttons: Right joystick y, left joystick x, right joystick x
import frc.robot.subsystems.OI.PRIMARYPADBUTTONS;
import frc.robot.subsystems.OI.SECONDARYPADBUTTONS;

public class OldOI extends SubsystemBase
{
  public enum BUTTONS{
    A(1),
    B(2),
    X(3),
    Y(4), 
    LeftJoystickY(1),
    LeftJoystickX(0),
    RightJoystickY(5),
    RightJoystickX(4),
    LeftJoystickPress(9),
    RightJoystickPress(10),
    //TODO Fix DPad
    // DPadUp(),
    // DPadLeft,
    // DPadDown,
    // DPadRight,
    LeftBumper(5),
    RightBumper(6), 
    LeftTrigger(2),
    RightTrigger(3),
    ViewButton(7),
    MenuButton(8);

    private int buttonValue;

    BUTTONS(int buttonValue){
      this.buttonValue = buttonValue;
    }
    public int getButtonVal(){
      return buttonValue;
    }
  }

  // Declares our controller variable
  public static Joystick driverController;
  public static Joystick operatorController;

  public Debouncer fieldCentricDebouncer = new Debouncer(0.05);
  public Debouncer parkingBrakeDebouncer = new Debouncer(0.05);
  public Debouncer menuDriverButtonDebouncer = new Debouncer(0.05);
  public Debouncer aDriverButtonDebouncer = new Debouncer(0.05);
  public Debouncer bDriverButtonDebouncer = new Debouncer(0.05);
  public Debouncer yDriverButtonDebouncer = new Debouncer(0.05);
  public Debouncer xDriverButtonDebouncer = new Debouncer(0.05);
  public Debouncer menuOperatorButtonDebouncer = new Debouncer(0.13);
  public Debouncer viewDriverButtonDebouncer = new Debouncer(0.05);

  // Declares the "zero" value variables (which allow us to compensate for joysticks that are a little off)
  private double LEFT_X_ZERO;
  private double LEFT_Y_ZERO;
  private double RIGHT_X_ZERO;
  private double RIGHT_Y_ZERO;

  /** Creates a new OI. */
  public OldOI() 
  {
    // Sets the driver controller to a new joystick object at port 0
    driverController = new Joystick(0);
    operatorController = new Joystick(1);
    zeroDriverController();
    zeroOperatorController();
  }

  /** This method will be called once per scheduler run */
  @Override
  public void periodic() 
  {    
    // You can add more smartdashboard printouts here for additional joysticks or buttons
  }

  public void zeroDriverController() 
  {
    //Sets all the offsets to zero, then uses whatever value it returns as the new offset.
    LEFT_X_ZERO = 0;
    LEFT_Y_ZERO = 0;
    RIGHT_X_ZERO = 0;
    RIGHT_Y_ZERO = 0;
    LEFT_X_ZERO = getDriverLeftX();
    LEFT_Y_ZERO = getDriverLeftY();
    RIGHT_X_ZERO = getDriverRightX();
    RIGHT_Y_ZERO = getDriverRightY();
  }

  /** The following methods return quality-controlled values from the driver controller */
  public double getDriverLeftX() 
  {
    // "Clamping" the value makes sure that it's still between 1 and -1 even if we have added an offset to it
    return MathUtil.clamp(driverController.getRawAxis(BUTTONS.LeftJoystickX.getButtonVal()) - LEFT_X_ZERO, -1, 1);
  }

  public double getDriverLeftY() 
  {
    return MathUtil.clamp(driverController.getRawAxis(BUTTONS.LeftJoystickY.getButtonVal()) - LEFT_Y_ZERO, -1, 1);
  }

  public double getDriverRightX() 
  {
    return MathUtil.clamp(driverController.getRawAxis(BUTTONS.RightJoystickX.getButtonVal()) - RIGHT_X_ZERO, -1, 1);
  }

  public double getDriverRightY() 
  {
    return MathUtil.clamp(driverController.getRawAxis(BUTTONS.RightJoystickY.getButtonVal()) - RIGHT_Y_ZERO, -1, 1);
  }

  public double getDriverTranslateX()
  {
    return getDriverLeftX();
  }

  public double getDriverTranslateY()
  {
    return getDriverLeftY();
  }

  public double getDriverRotate()
  {
    return getDriverRightX();
  }

  public double getDriverRightTrigger()
  {
    return driverController.getRawAxis(BUTTONS.RightTrigger.getButtonVal());
  }

  public double getDriverLeftTrigger()
  {
    return driverController.getRawAxis(BUTTONS.LeftTrigger.getButtonVal());
  }

  public boolean getDriverLeftBumper(){
    return parkingBrakeDebouncer.calculate(driverController.getRawButton(BUTTONS.LeftBumper.getButtonVal()));
  }

  public boolean getDriverRightBumper(){
    return fieldCentricDebouncer.calculate(driverController.getRawButton(BUTTONS.RightBumper.getButtonVal()));
  }

  /** Returns a specified button from the driver controller */
  public boolean getDriverRawButton(int i) 
  {
    return driverController.getRawButton(i);
  }

  public boolean getDriverAButton(){
    return aDriverButtonDebouncer.calculate(driverController.getRawButton(BUTTONS.A.getButtonVal()));
  }

  public boolean getDriverBButton(){
    return bDriverButtonDebouncer.calculate(driverController.getRawButton(BUTTONS.B.getButtonVal()));
  }

  public boolean getDriverXButton()
  {
    return xDriverButtonDebouncer.calculate(driverController.getRawButton(BUTTONS.X.getButtonVal()));
  }

  public boolean getDriverYButton()
  {
    return yDriverButtonDebouncer.calculate(driverController.getRawButton(BUTTONS.Y.getButtonVal()));
  }

  public boolean getDriverMenuButton(){
    return menuDriverButtonDebouncer.calculate(driverController.getRawButton(BUTTONS.MenuButton.getButtonVal()));
  }

  public boolean getDriverViewButton(){
    return viewDriverButtonDebouncer.calculate(driverController.getRawButton(BUTTONS.ViewButton.getButtonVal()));
  }

  public boolean getDriverDPadUp()
  {
    return (driverController.getPOV() == 0);
  }

  public boolean getDriverDPadDown()
  {
    return (driverController.getPOV() == 180);
  }

  public boolean getDriverDPadLeft()
  {
    return (driverController.getPOV() == 270);
  }

  public boolean getDriverDPadRight()
  {
    return (driverController.getPOV() == 90);
  }

  //TODO William-What is this?????
  public boolean getDriverAlignButtons()
  {
    return getDriverAButton() || getDriverViewButton() || getDriverXButton() || getDriverYButton();
  }

  public boolean getDriverLeftJoystickPress(){
    return getDriverRawButton(BUTTONS.LeftJoystickPress.getButtonVal());
  }

  public void rumble() {
    // OI.driverController.setRumble(RumbleType.kBothRumble, 1);
  }

  public void stopRumble() {
    // OI.driverController.setRumble(RumbleType.kBothRumble, 0);
  }

  public void zeroOperatorController() {
    //Sets all the offsets to zero, then uses whatever value it returns as the new offset.
    LEFT_X_ZERO = 0;
    LEFT_Y_ZERO = 0;
    RIGHT_X_ZERO = 0;
    RIGHT_Y_ZERO = 0;
    LEFT_X_ZERO = getOperatorLeftX();
    LEFT_Y_ZERO = getOperatorLeftY();
    RIGHT_X_ZERO = getOperatorRightX();
    RIGHT_Y_ZERO = getOperatorRightY();
  }

  /** The following methods return quality-controlled values from the operator controller */
  public double getOperatorLeftX() {
    if(Math.abs(operatorController.getRawAxis(BUTTONS.LeftJoystickX.getButtonVal())) < 0.1){
      return 0.0;
    }
    // "Clamping" the value makes sure that it's still between 1 and -1 even if we have added an offset to it
    return MathUtil.clamp(operatorController.getRawAxis(BUTTONS.LeftJoystickX.getButtonVal()) - LEFT_X_ZERO, -1, 1);
  }

  public double getOperatorLeftY() {
    if(Math.abs(operatorController.getRawAxis(BUTTONS.LeftJoystickY.getButtonVal())) < 0.1){
      return 0.0;
    }
    return -1.0 * MathUtil.clamp(operatorController.getRawAxis(BUTTONS.LeftJoystickY.getButtonVal()) - LEFT_Y_ZERO, -1, 1);
  }

  public double getOperatorRightX() {
    if(Math.abs(operatorController.getRawAxis(BUTTONS.RightJoystickX.getButtonVal())) < 0.1){
      return 0.0;
    }
    return MathUtil.clamp(operatorController.getRawAxis(BUTTONS.RightJoystickX.getButtonVal()) - RIGHT_X_ZERO, -1, 1);
  }

  public double getOperatorRightY() {
    if(Math.abs(operatorController.getRawAxis(BUTTONS.RightJoystickY.getButtonVal())) < 0.1){
      return 0.0;
    }
    return -1.0 * MathUtil.clamp(operatorController.getRawAxis(BUTTONS.RightJoystickY.getButtonVal()) - RIGHT_Y_ZERO, -1, 1);
  }

  public boolean getOperatorRightBumper(){
    return getOperatorRawButton(BUTTONS.RightBumper.getButtonVal());
  }

  public boolean getOperatorLeftBumper(){
    return getOperatorRawButton(BUTTONS.LeftBumper.getButtonVal());
  }

  /** Returns a specified button from the operator controller */
  public boolean getOperatorRawButton(int i) {
    return operatorController.getRawButton(i);
  }

  public boolean getOperatorAButton(){
    return getOperatorRawButton(BUTTONS.A.getButtonVal());
  }

  public boolean getOperatorBButton(){
    return getOperatorRawButton(BUTTONS.B.getButtonVal());
  }

  public boolean getOperatorXButton(){
    return getOperatorRawButton(BUTTONS.X.getButtonVal());
  }

  public boolean getOperatorYButton(){
    return getOperatorRawButton(BUTTONS.Y.getButtonVal());
  }

  public double getOperatorRightTrigger(){
    return MathUtil.clamp(operatorController.getRawAxis(BUTTONS.RightTrigger.getButtonVal()), 0, 1);
  }

  public double getOperatorLeftTrigger(){
    return MathUtil.clamp( operatorController.getRawAxis(BUTTONS.LeftTrigger.getButtonVal()), 0, 1);
  }

  public boolean getOperatorViewButton() {
    return getOperatorRawButton(BUTTONS.ViewButton.getButtonVal());
  }

  public boolean getOperatorMenuButton() {
    return menuOperatorButtonDebouncer.calculate(operatorController.getRawButton(BUTTONS.MenuButton.getButtonVal()));

  }

  public boolean getOperatorDPadUp(){
    return (operatorController.getPOV() == 0);
  }

  public boolean getOperatorDPadDown(){
    return (operatorController.getPOV() == 180);
  }

  public boolean getOperatorDPadLeft(){
    return (operatorController.getPOV() == 270);
  }

  public boolean getOperatorDPadRight(){
    return (operatorController.getPOV() == 90);
  }

  public boolean getOperatorLeftJoystickPress(){
    return getOperatorRawButton(BUTTONS.LeftJoystickPress.getButtonVal());
  }

  public boolean getOperatorRightJoystickPress(){
    return getOperatorRawButton(BUTTONS.RightJoystickPress.getButtonVal());
  }

  @Override
  public void initSendable(SendableBuilder builder){
    builder.setSmartDashboardType("OI");
    builder.addDoubleProperty("Driver Right Y", this::getDriverRightY, null);
    builder.addDoubleProperty("Driver Right X", this::getDriverRightX, null);
    builder.addDoubleProperty("Driver Left Y", this::getDriverLeftY, null);
    builder.addDoubleProperty("Driver Left X", this::getDriverLeftX, null);
    builder.addDoubleProperty("Operator Right Y", this::getOperatorRightY, null);
    builder.addDoubleProperty("Operator Right X", this::getOperatorRightX, null);
    builder.addDoubleProperty("Operator Left Y", this::getOperatorLeftY, null);
    builder.addDoubleProperty("Operator Left X", this::getOperatorLeftX, null);
  }
}