// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.XboxController;
//import frc.robot.Constants;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

public class CANdleControl extends SubsystemBase {
  CANdle m_candle;
  int numTotalLED = 58;
  int numPerStrip = 13;
  int candleNum = 8;

  public CANdleControl() {
    m_candle = new CANdle(30); // creates a new CANdle with ID 0
    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGB; // set the strip type to RGB
    config.brightnessScalar = 0.5; // dim the LEDs to half brightness
    m_candle.configAllSettings(config);
    m_candle.configLEDType(LEDStripType.GRB, 5);
    m_candle.setLEDs(255, 255, 255); // set the CANdle LEDs to white
    this.clearAnim();// CLEARS ANIMATIONS
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * sets rgb to a color
   * @param r amount of red
   * @param g amount of green
   * @param b amount of blue
   * @param q starting led number
   * @param c number of total leds
   */
  public void setRGB(int r, int g, int b, int q, int c){
    m_candle.setLEDs(r, g, b, 0, q, c);
  }

  public int getTotalLED(){
    return numTotalLED;
  }

  public int getStripLED(){
    return numPerStrip;
  }

  public int getCandleNum() {
    return candleNum;
  }

  /**
   * creates rainbow animation
   * @param n number of leds
   * @param s which led to start at
   */
  public void setRainbow(int n, int s){
    RainbowAnimation rainbowAnim = new RainbowAnimation(1, 0.5, n, false, s);
    m_candle.animate(rainbowAnim);
  }

  /**
   * clears current led animation
   */
  public void clearAnim(){
    m_candle.clearAnimation(0);// CLEARS ANIMATIONS
  }
}