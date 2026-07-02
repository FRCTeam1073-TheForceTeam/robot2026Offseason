// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.TreeSet;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.DashboardNames;

public class ZoneFinder extends SubsystemBase
{
    public static class Zone
    {
        public final String name;
        public final Rectangle2d rect;

        public Zone(String name, Rectangle2d rect)
        {
            this.name = name;
            this.rect = rect;
        }
    }

    private final Localizer localizer;
    private Optional<Alliance> alliance;
    private Translation2d currentTrans = new Translation2d();
    private final List<Zone> zones = new ArrayList<>();

    public ZoneFinder(Localizer localizer)
    {
        this.localizer = localizer;

        // Welded Dimensions
        zones.add(new Zone("BLUEZONE", new Rectangle2d(in(0, 0), in(156.61, 317.69))));
        zones.add(new Zone("REDZONE", new Rectangle2d(in(490.01, 0), in(651.22, 317.69))));
        zones.add(new Zone("NEUTRALZONE", new Rectangle2d(in(201.01, 0), in(325.61, 317.69))));

        // Blue alliance POV
        zones.add(new Zone("RIGHTHALF", new Rectangle2d(in(0, 0), in(651.22, 158.84))));
        zones.add(new Zone("LEFTHALF", new Rectangle2d(in(0, 158.84), in(651.22, 317.69))));

        // added 35 in. (robot dimensions w/ bumpers) in x-dimension on either side to expand trench zones. -35 to first x-value, +35 to second x-value
        zones.add(new Zone("TRENCH", new Rectangle2d(in(121.61, 0), in(236.01, 50.35))));
        zones.add(new Zone("TRENCH", new Rectangle2d(in(121.61, 267.098), in(236.01, 317.69))));
        zones.add(new Zone("TRENCH", new Rectangle2d(in(410.61, 0), in(525.01, 50.35))));
        zones.add(new Zone("TRENCH", new Rectangle2d(in(410.61, 267.098), in(525.01, 317.69))));

        zones.add(new Zone("BUMP", new Rectangle2d(in(156.61, 62.59), in(201.01, 135.59))));
        zones.add(new Zone("BUMP", new Rectangle2d(in(156.61, 182.1), in(201.01, 255.1))));
        zones.add(new Zone("BUMP", new Rectangle2d(in(445.61, 62.59), in(490.01, 135.59))));
        zones.add(new Zone("BUMP", new Rectangle2d(in(445.61, 182.1), in(490.01, 255.1))));
    }

    private static Translation2d in(double xInches, double yInches)
    {
        return new Translation2d(Units.inchesToMeters(xInches), Units.inchesToMeters(yInches));
    }

    public Set<String> getZones()
    {
        Set<String> result = new TreeSet<>();
        for (Zone zone : zones) {
            if (zone.rect.contains(currentTrans)) {
                result.add(zone.name);
            }
        }
        return result;
    }

    @Override
    public void periodic()
    {
        alliance = DriverStation.getAlliance();
        currentTrans = localizer.getPose().getTranslation();
        Set<String> result = getZones();

        alliance = DriverStation.getAlliance();
        StringBuilder zonelist = new StringBuilder();
        for (String zone : result) {
            String zonePortion = zone;
            // this part switches left and right for red alliance POV since all code is from blue alliance POV
            if (alliance.get() == Alliance.Red && zone.contains("RIGHTHALF")) {
                zonePortion = "LEFTHALF";
            } else if (alliance.get() == Alliance.Red && zone.contains("LEFTHALF")) {
                zonePortion = "RIGHTHALF";
            }
            zonelist.append(zonePortion).append(", ");
        }
        SmartDashboard.putString(DashboardNames.ZONE_ZONE.getKey(), zonelist.toString());
    }
}
