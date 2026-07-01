// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TargetFinder extends SubsystemBase
{
    // TODO: Fix.
    public static final Pose2d FIELD_CENTER = new Pose2d(Units.inchesToMeters(291.00), Units.inchesToMeters(158.32), new Rotation2d());

    // Welded Perimeter
    public static final Pose2d BLUEHUB = new Pose2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84), new Rotation2d());
    public static final Pose2d REDHUB = new Pose2d(Units.inchesToMeters(469.11), Units.inchesToMeters(158.84), new Rotation2d());

    public static final Transform2d ROBOTOTURRET = new Transform2d(Units.inchesToMeters(-4.391), Units.inchesToMeters(-7.409), new Rotation2d());

    // literally the most estimated values ever
    public static final Pose2d REDPASS_R = new Pose2d(Units.inchesToMeters(557.5), Units.inchesToMeters(237.5), new Rotation2d());
    public static final Pose2d REDPASS_L = new Pose2d(Units.inchesToMeters(557.5), Units.inchesToMeters(80), new Rotation2d());
    public static final Pose2d BLUEPASS_R = new Pose2d(Units.inchesToMeters(91), Units.inchesToMeters(80), new Rotation2d());
    public static final Pose2d BLUEPASS_L = new Pose2d(Units.inchesToMeters(91), Units.inchesToMeters(237.5), new Rotation2d());

    private final Localizer localizer;
    private final ZoneFinder zonefinder;
    private BallisticShot ballisticShot;

    private Pose2d ourHub;
    private Pose2d turretPos = new Pose2d();
    private Pose2d target;
    private boolean passing = false;

    private Optional<Alliance> alliance;
    private Set<String> zone;
    private String ourZone;

    private double turretToTargetAngle = 0.0;
    private double rangeToTarget = 0.0;

    public TargetFinder(Localizer localizer, ZoneFinder zonefinder)
    {
        this.localizer = localizer;
        this.zonefinder = zonefinder;
    }

    public void setBallisticShot(BallisticShot ballisticShot)
    {
        this.ballisticShot = ballisticShot;
    }

    public Pose2d getTargetPos()
    {
        if (zonefinder.getZones().contains(ourZone)) {
            target = getHubPos();
            passing = false;
        } else {
            target = pass();
            passing = true;
        }

        return target;
    }

    public Pose2d getHubPos()
    {
        // All in field coordinates
        Pose2d targetLoc = ourHub;

        BallisticShot.Shot shot = ballisticShot.getShot();
        // TODO: Use cross-product to compute relative velocity induced by rotation and add that term as well...
        Translation2d velocityOffset = new Translation2d(
            localizer.getSpeeds().vxMetersPerSecond * shot.shotTime,
            localizer.getSpeeds().vyMetersPerSecond * shot.shotTime);

        targetLoc = targetLoc.transformBy(new Transform2d(velocityOffset.unaryMinus(), new Rotation2d()));

        // turns into robot coordinates
        return targetLoc.relativeTo(turretPos);
    }

    // right and left are swapped for red alliance bc zones are from blue alliance perspective
    public Pose2d pass()
    {
        updateAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Red && zone.contains("RIGHTHALF")) {
                return REDPASS_L.relativeTo(turretPos);
            } else if (alliance.get() == Alliance.Red && zone.contains("LEFTHALF")) {
                return REDPASS_R.relativeTo(turretPos);
            } else if (alliance.get() == Alliance.Blue && zone.contains("RIGHTHALF")) {
                return BLUEPASS_R.relativeTo(turretPos);
            } else if (alliance.get() == Alliance.Blue && zone.contains("LEFTHALF")) {
                return BLUEPASS_L.relativeTo(turretPos);
            }
        }

        // If we don't know anything else.
        return FIELD_CENTER.relativeTo(turretPos);
    }

    @Override
    public void periodic()
    {
        updateAlliance();

        zone = zonefinder.getZones();
        turretPos = localizer.getPose().transformBy(ROBOTOTURRET);

        Translation2d relativeTargetPos = getTargetPos().getTranslation();

        double angle = Math.atan2(relativeTargetPos.getY(), relativeTargetPos.getX());
        turretToTargetAngle = angle;
        rangeToTarget = relativeTargetPos.getNorm();
        SmartDashboard.putNumber("TargetFinder/Turret Angle", turretToTargetAngle);
        SmartDashboard.putNumber("TargetFinder/Turret Range", rangeToTarget);
    }

    public double getTurretToTargetAngleRadians()
    {
        return turretToTargetAngle;
    }

    public double getRangeToTargetMeters()
    {
        return rangeToTarget;
    }

    public boolean isPassing()
    {
        return passing;
    }

    private void updateAlliance()
    {
        alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Red) {
                ourHub = REDHUB;
                ourZone = "REDZONE";
            } else if (alliance.get() == Alliance.Blue) {
                ourHub = BLUEHUB;
                ourZone = "BLUEZONE";
            }
        } else {
            System.out.println("TargetFinder::No Alliance Selected");
        }
    }
}
