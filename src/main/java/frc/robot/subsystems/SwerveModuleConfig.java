// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;


/** Add your docs here. */
public class SwerveModuleConfig
{
    public int moduleNumber = -1;
    public double gearRatio = 6.03;
    public double wheelDiameterMeters = 0.1016;
    public Translation2d position = new Translation2d(0,0);
    public double rotationsPerMeter = gearRatio / (wheelDiameterMeters * Math.PI);
    public double steerGearRatio = 287.0 / 11.0;
    public double steerCurrentLimit = 25;
    public double driveCurrentLimit = 40;
    public double steerVoltageLimit = 8.5;
    public double driveVoltageLimit = 8.5;
    public double steerP = 0;
    public double steerI = 0;
    public double steerD = 0;
    public double steerV = 0;
    public double steerS = 0;
    public double driveP = 0;
    public double driveI = 0;
    public double driveD = 0;
    public double driveV = 0;
    public double driveA = 0;
    public double driveS = 0;


    /**SwerveModuleConfig contstructor sets PIDF values and current limits
     *
     */
    public SwerveModuleConfig()
    {
        driveP = 0.35;
        driveI = 0.0;
        driveD = 0.0;
        driveV = 0.12;
        driveA = 0.0;
        driveS = 0.015;

        steerP = 11.0;
        steerI = 0.8;
        steerD = 0.04;
        steerV = 0.153;
        steerS = 0.04;
    }
}