// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LaserCan extends SubsystemBase
{
  private final au.grapplerobotics.LaserCan laserCan;

  private boolean isValid = false;
  private double rangeMeters = 0.0;

  public LaserCan()
  {
    setName("LaserCan");

    laserCan = new au.grapplerobotics.LaserCan(28);

    boolean hardwareConfigured = configureHardware();
    if (!hardwareConfigured) {
      System.err.println("LaserCAN: Hardware Failed To Configure!");
    }
    SmartDashboard.putBoolean("LaserCan/LaserCAN - hardware_configured", hardwareConfigured);
  }

  private boolean configureHardware()
  {
    try {
      laserCan.setRangingMode(RangingMode.LONG); // TODO: set ranging mode
      laserCan.setTimingBudget(TimingBudget.TIMING_BUDGET_100MS); // TODO: set timing budget
      laserCan.setRegionOfInterest(new RegionOfInterest(8, 8, 16, 16)); // TODO: change values
      return true;
    } catch (ConfigurationFailedException e) {
      return false;
    }
  }

  @Override
  public void periodic()
  {
    Measurement measurementData = laserCan.getMeasurement();
    if (measurementData != null && measurementData.status == au.grapplerobotics.interfaces.LaserCanInterface.LASERCAN_STATUS_VALID_MEASUREMENT) {
      isValid = true;
      rangeMeters = measurementData.distance_mm / 1000.0;
    } else {
      isValid = false;
      rangeMeters = 0.0;
    }

    SmartDashboard.putBoolean("LaserCan/is_valid", isValid);
    SmartDashboard.putNumber("LaserCan/distance(m)", rangeMeters);
  }

  public boolean isValid()
  {
    return isValid;
  }

  public double getRangeMeters()
  {
    return rangeMeters;
  }
}
