# Swerve Drive Veering Analysis

## Problem Statement
When the driver pushes the joystick straight forward, the robot moves in that direction but veers off to the side.

---

## 🔴 Critical Issue: CANcoder Magnetic Offsets Not Set

**Location:** [SwerveModule.java:28-34](src/main/java/frc/robot/subsystems/SwerveModule.java#L28-L34)

**Issue:**
```java
/** Swerve module class one for each swerve module. 
 * 
 * ! ! ! ! ! NOTE ! ! ! ! ! ! !
 *  
 * THIS CODE ONLY WORKS IF YOU SET THE CANCODERS MAGNETIC OFFSET TO BOOT TO ABSOLUTE POSITION
 * 
 * OTHERWISE THE WHEELS WILL NOT INITIALIZE IN THE CORRECT POSITIONS
 * 
 * ! ! ! ! ! NOTE ! ! ! ! ! ! !
*/
```

If the magnetic offsets aren't programmed into each CANcoder, your wheels won't be at their true reference angles when the robot starts. **This is the #1 cause of veering in swerve drives.**

### Why This Matters
- The steering control loop reads `steerEncoder.getAbsolutePosition()` [SwerveModule.java:85](src/main/java/frc/robot/subsystems/SwerveModule.java#L85)
- If the CANcoder doesn't know its magnetic offset, it can't report the true absolute angle
- All 4 wheels might be pointing in slightly different directions even though you commanded them all to 0°
- This causes the robot to veer sideways when driving forward

### Solution
Program each CANcoder with its magnetic offset using CTRE's Phoenix Tuner (or similar tool):
1. Mount all wheels to point straight ahead on the robot
2. For each CANcoder, read its current raw value
3. Calculate the magnetic offset = (0.0 - raw_value)
4. Program that offset into the CANcoder's configuration to "boot to absolute position"
5. Verify that all CANcoders now read `0.0 rotations` when wheels point straight ahead

**CANcoder IDs:** [SwerveModuleIDConfig.java](src/main/java/frc/robot/subsystems/SwerveModuleIDConfig.java)
- Module 0 (Front-Left): Encoder ID 6
- Module 1 (Front-Right): Encoder ID 9
- Module 2 (Back-Left): Encoder ID 12
- Module 3 (Back-Right): Encoder ID 15

---

## ⚠️ Likely Issue: Drive Motor Characterization Mismatch

**Location:** [SwerveModuleConfig.java:40-46](src/main/java/frc/robot/subsystems/SwerveModuleConfig.java#L40-L46)

**Issue:**
```java
driveV = 0.12;    // Velocity feedforward (same for all modules)
driveS = 0.015;   // Static friction (same for all modules)
```

All 4 modules use identical feedforward values. If even one module has slightly different mechanical resistance (bearing friction, gear efficiency, etc.), it will produce different speed than the others, causing veering.

### Why This Matters
- The drive motors use velocity control with kV feedforward: `driveMotor.setControl(driveVelocityVoltage.withVelocity(...))`
- If Module A requires 0.12V/rps but Module B only needs 0.10V/rps, they'll accelerate at different rates
- On straight-line commands, this mismatch accumulates and pushes the robot sideways

### How to Check
Run the `setDebugSpeed()` command with all modules at the same target velocity. Monitor the actual velocities:
```java
public void setDebugSpeed(double speed) {
    modules[0].setDriveVelocity(speed);
    modules[1].setDriveVelocity(speed);
    modules[2].setDriveVelocity(speed);
    modules[3].setDriveVelocity(speed);
}
```

If velocities differ by >5%, you need to retune the drive characterization. Run a SysID characterization for each module to get module-specific kV and kS values, then add per-module tuning if needed.

---

## ⚠️ Possible Issue: Encoder Reading Offset

**Location:** [SwerveModule.java:82-86](src/main/java/frc/robot/subsystems/SwerveModule.java#L82-L86)

**Issue:**
```java
public double getSteerRotations() {
    steerEncoder.getAbsolutePosition().refresh();
    return ((steerEncoder.getAbsolutePosition().getValueAsDouble()));
}
```

This reads the steering angle directly from the CANcoder. If a CANcoder's magnetic offset is wrong, this will return an incorrect angle, causing the steering controller to aim at the wrong target.

### Workaround (Not Recommended)
You *could* add a per-module offset here to compensate:
```java
return steerEncoder.getAbsolutePosition().getValueAsDouble() + moduleOffsets[cfg.moduleNumber];
```

However, **this is a band-aid**. The real fix is to set the CANcoder magnetic offsets correctly.

---

## Diagnosis Checklist

Before making changes, verify:

- [ ] **CANcoder offsets programmed?** Use Phoenix Tuner to check each encoder's MagnetOffset value (should not be 0.0)
- [ ] **Wheels point straight?** Manually align all wheels to 0° and verify CANcoders read 0.0 rotations
- [ ] **Drive motors balanced?** Run `setDebugSpeed()` and check if all 4 modules reach the same velocity
- [ ] **Motor inversions consistent?** All motors set to `CounterClockwise_Positive` [SwerveModule.java:240](src/main/java/frc/robot/subsystems/SwerveModule.java#L240)
- [ ] **Kinematics positions symmetric?** Module positions are correct [Drivetrain.java:69-98](src/main/java/frc/robot/subsystems/Drivetrain.java#L69-L98)

---

## Implementation Order

1. **Start here:** Set CANcoder magnetic offsets (most likely culprit)
2. **If still veering:** Check drive motor characterization and rebalance kV/kS
3. **If still veering:** Verify module positions and motor inversions
4. **Last resort:** Add per-module steering offset compensation (temporary)

---

## Related Code

- **Drivetrain controller:** [Drivetrain.java:327-338](src/main/java/frc/robot/subsystems/Drivetrain.java#L327-L338) — where steering angles and drive speeds are commanded
- **Module control loop:** [SwerveModule.java:124-155](src/main/java/frc/robot/subsystems/SwerveModule.java#L124-L155) — cosine compensator and coupling offset
- **Steering configuration:** [SwerveModule.java:199-229](src/main/java/frc/robot/subsystems/SwerveModule.java#L199-L229) — PID and sensor setup
- **Drive configuration:** [SwerveModule.java:231-257](src/main/java/frc/robot/subsystems/SwerveModule.java#L231-L257) — velocity control setup
