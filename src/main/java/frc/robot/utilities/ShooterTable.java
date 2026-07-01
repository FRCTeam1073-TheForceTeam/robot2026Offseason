// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterTable
{
    // const double SPEED_SCALE = 0.96; AndyMark At GSD
    private static final double SPEED_SCALE = 0.975;

    private final InterpolatingDoubleTreeMap hoodTable = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap flywheelTable = new InterpolatingDoubleTreeMap();

    public ShooterTable()
    {
        hoodTable.put(1.00, Math.toRadians(69.2));
        hoodTable.put(1.50, Math.toRadians(65.64));
        hoodTable.put(2.00, Math.toRadians(63.81));
        hoodTable.put(2.30, Math.toRadians(62.89));
        hoodTable.put(2.72, Math.toRadians(62.32));
        hoodTable.put(3.00, Math.toRadians(61.17));
        hoodTable.put(3.38, Math.toRadians(59.46));
        hoodTable.put(3.85, Math.toRadians(57.74));
        hoodTable.put(4.20, Math.toRadians(55.74));
        hoodTable.put(4.76, Math.toRadians(53.15));
        hoodTable.put(5.04, Math.toRadians(53.9));
        hoodTable.put(6.00, Math.toRadians(41.7)); // this is for passing

        flywheelTable.put(1.00, 8.00 * SPEED_SCALE);
        flywheelTable.put(1.50, 8.20 * SPEED_SCALE);
        flywheelTable.put(2.00, 8.80 * SPEED_SCALE);
        flywheelTable.put(2.30, 9.00 * SPEED_SCALE);
        flywheelTable.put(2.72, 9.20 * SPEED_SCALE);
        flywheelTable.put(3.00, 9.40 * SPEED_SCALE);
        flywheelTable.put(3.38, 9.80 * SPEED_SCALE);
        flywheelTable.put(3.85, 10.0 * SPEED_SCALE);
        flywheelTable.put(4.20, 10.4 * SPEED_SCALE);
        flywheelTable.put(4.76, 10.6 * SPEED_SCALE);
        flywheelTable.put(5.045656, 10.5 * SPEED_SCALE); // 198.64in
        flywheelTable.put(6.00, 9.0); // this is for passing
        flywheelTable.put(7.00, 14.0); // this is for passing
    }

    public double getHoodAngle(double rangeMeters)
    {
        return hoodTable.get(rangeMeters);
    }

    public double getFlywheelVelocity(double rangeMeters)
    {
        return flywheelTable.get(rangeMeters);
    }
}
