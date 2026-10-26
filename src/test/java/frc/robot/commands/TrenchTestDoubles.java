package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.subsystems.BallisticShot;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.TargetFinder;
import frc.robot.subsystems.ZoneFinder;
import frc.robot.utilities.ShooterTable;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Shared test doubles for the TRENCH-zone command tests. Each double records what a command asked for
 * or returns values the test sets directly.
 */
final class TrenchTestDoubles {
    private TrenchTestDoubles() {
    }

    /**
     * Sets the alliance the driver station reports, then checks that DriverStation picked it up. Tests that call this
     * should reset the driver station in an @AfterEach, since DriverStationSim state is shared across tests.
     */
    static void setAlliance(AllianceStationID station, Alliance expected) {
        DriverStationSim.setAllianceStationId(station);
        DriverStationSim.notifyNewData();
        assertEquals(expected, DriverStation.getAlliance().orElse(null),
            "DriverStationSim should make DriverStation report the alliance under test");
    }

    /** Clears the driver station's alliance, then checks that DriverStation reports none. */
    static void clearAlliance() {
        DriverStationSim.resetData();
        DriverStationSim.notifyNewData();
        assertTrue(DriverStation.getAlliance().isEmpty(),
            "DriverStationSim should make DriverStation report no alliance");
    }

    /** Intake double that records the commands IntakeTeleop issues instead of driving hardware. */
    static class RecordingIntake extends Intake {
        double position = 0.0;
        double lastSetPosition = Double.NaN;
        boolean stopCalled = false;
        final List<Double> commandedPositions = new ArrayList<>();

        @Override
        public void setPosition(double radians) {
            lastSetPosition = radians;
            commandedPositions.add(radians);
        }

        @Override
        public void stop() {
            stopCalled = true;
        }

        @Override
        public double getPositionRadians() {
            return position;
        }
    }

    /** Climber double that records the position commands ClimberTeleop issues. */
    static class RecordingClimber extends Climber {
        double position = 0.0;
        double lastSetPosition = Double.NaN;
        boolean stopCalled = false;

        @Override
        public void setPosition(double meters) {
            lastSetPosition = meters;
        }

        @Override
        public void stop() {
            stopCalled = true;
        }

        @Override
        public double getClimberPosition() {
            return position;
        }

        void reset() {
            position = 0.0;
            lastSetPosition = Double.NaN;
            stopCalled = false;
        }
    }

    /** Shooter hood double that records the position commands HoodTeleop issues. */
    static class RecordingShooterHood extends ShooterHood {
        double lastSetPosition = Double.NaN;
        boolean stopCalled = false;

        @Override
        public void setPosition(double radians) {
            lastSetPosition = radians;
        }

        @Override
        public void stop() {
            stopCalled = true;
        }
    }

    /** OI double whose buttons and triggers are set directly by each test. Everything starts released. */
    static class FakeOI extends OI {
        boolean driverRightBumper = false;
        boolean driverAButton = false;
        boolean driverXButton = false;
        boolean driverMenuButton = false;
        boolean driverViewButton = false;
        boolean operatorXButton = false;
        boolean operatorYButton = false;
        boolean operatorMenuButton = false;
        boolean operatorViewButton = false;
        double operatorLeftTrigger = 0.0;
        boolean ballisticShotMode = false;

        @Override
        public boolean getDriverRightBumper() {
            return driverRightBumper;
        }

        @Override
        public boolean getDriverAButton() {
            return driverAButton;
        }

        @Override
        public boolean getDriverXButton() {
            return driverXButton;
        }

        @Override
        public boolean getDriverMenuButton() {
            return driverMenuButton;
        }

        @Override
        public boolean getDriverViewButton() {
            return driverViewButton;
        }

        @Override
        public boolean getOperatorXButton() {
            return operatorXButton;
        }

        @Override
        public boolean getOperatorYButton() {
            return operatorYButton;
        }

        @Override
        public boolean getOperatorMenuButton() {
            return operatorMenuButton;
        }

        @Override
        public boolean getOperatorViewButton() {
            return operatorViewButton;
        }

        @Override
        public double getOperatorLeftTrigger() {
            return operatorLeftTrigger;
        }

        @Override
        public boolean ballisticShotMode() {
            return ballisticShotMode;
        }
    }

    /** ZoneFinder double whose zones are set directly by each test. */
    static class FakeZoneFinder extends ZoneFinder {
        final Set<String> zones = new HashSet<>();

        FakeZoneFinder() {
            super(null);
        }

        @Override
        public Set<String> getZones() {
            return zones;
        }
    }

    /** TargetFinder double with a settable range and passing state. */
    static class FakeTargetFinder extends TargetFinder {
        double rangeMeters = 3.0;
        boolean passing = false;

        FakeTargetFinder() {
            super(null, null);
        }

        @Override
        public double getRangeToTargetMeters() {
            return rangeMeters;
        }

        @Override
        public boolean isPassing() {
            return passing;
        }
    }

    /** BallisticShot double that returns a fixed hood angle. */
    static class FakeBallisticShot extends BallisticShot {
        double hoodAngle = 0.0;

        FakeBallisticShot(TargetFinder targetFinder) {
            super(targetFinder);
        }

        @Override
        public Shot getShot() {
            return new Shot(0.0, hoodAngle, 0.0);
        }
    }

    /** ShooterTable double that returns a fixed lookup-table hood angle. */
    static class FakeShooterTable extends ShooterTable {
        double hoodAngle = 0.0;

        @Override
        public double getHoodAngle(double rangeMeters) {
            return hoodAngle;
        }
    }
}
