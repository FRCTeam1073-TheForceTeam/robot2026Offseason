package frc.robot.utilities;

public enum DashboardNames {
  // Auto
  AUTO_LEVEL_CHOOSER("Level Chooser"),
  AUTO_START_DELAY("Start Delay(s)"),
  AUTO_PUT_INTAKE_OUT("Autos/Put Intake Out"),
  AUTO_START_TIME("Autos/Start Auto"),
  AUTO_GRABBED_CHOREO("Autos/Grabed Choreo"),
  AUTO_HAVE_TRAJECTORY("Autos/Have Trajectory"),
  AUTO_TRAJECTORY("Autos/Trajectory"),
  AUTO_WINNERS("Auto Winners"),

  // Drive
  DRIVE_SHIFT_TIME("Shift Time"),
  DRIVE_HAS_TRAJECTORY("Has Trajectory"),

  // Hub
  HUB_ACTIVE("Hub Active"),

  // Teleop Drive
  TELEOP_DRIVE_DPAD_DOWN("TeleopDrive/Driver DPad Down"),
  TELEOP_DRIVE_DPAD_LEFT("TeleopDrive/Driver DPad Left"),
  TELEOP_DRIVE_DPAD_RIGHT("TeleopDrive/Driver DPad Right"),
  TELEOP_DRIVE_DPAD_UP("TeleopDrive/Driver DPad Up"),
  TELEOP_DRIVE_DPAD_ANGLE("TeleopDrive/Driver DPad angle"),
  TELEOP_DRIVE_FAST_ROTATION("TeleopDrive/Fast Rotation"),
  TELEOP_DRIVE_FIELD_CENTRIC("TeleopDrive/FieldCentric"),
  TELEOP_DRIVE_SLOW_MODE("TeleopDrive/Slow Mode"),
  TELEOP_DRIVE_AVG_TORQUE("TeleopDrive/AvgTorque"),
  TELEOP_DRIVE_LEFT_X("TeleopDrive/leftX"),
  TELEOP_DRIVE_LEFT_Y("TeleopDrive/leftY"),
  TELEOP_DRIVE_MAX_ROTATION_VELOCITY("TeleopDrive/Maximum Rotation Velocity"),
  TELEOP_DRIVE_OMEGA("TeleopDrive/omega"),
  TELEOP_DRIVE_RIGHT_X("TeleopDrive/rightX"),
  TELEOP_DRIVE_VX("TeleopDrive/vx"),
  TELEOP_DRIVE_VY("TeleopDrive/vy"),

  // Drive Path
  DRIVE_PATH_CURRENT_SAMPLE("DrivePath/Current Sample"),
  DRIVE_PATH_END("DrivePath/End"),
  DRIVE_PATH_PAST_TIME("DrivePath/Past Time"),
  DRIVE_PATH_QUIT("DrivePath/Quit"),
  DRIVE_PATH_TRAJECTORY("DrivePath/Trajectory"),
  DRIVE_PATH_COMMANDED_VW("DrivePath/CommandedVw"),
  DRIVE_PATH_COMMANDED_VX("DrivePath/CommandedVx"),
  DRIVE_PATH_COMMANDED_VY("DrivePath/CommandedVy"),
  DRIVE_PATH_CURRENT_TIME("DrivePath/CurrentTime"),
  DRIVE_PATH_MAX_VELOCITY("DrivePath/MaxVelocity"),
  DRIVE_PATH_TARGET_THETA("DrivePath/TargetTheta"),
  DRIVE_PATH_TARGET_X("DrivePath/TargetX"),
  DRIVE_PATH_TARGET_Y("DrivePath/TargetY"),
  DRIVE_PATH_STATUS("DrivePath/Status"),

  // Test
  TEST_FLYWHEEL_LEVEL("TestFlywheel/level"),
  TEST_FLYWHEEL_SPEED("TestFlywheel/speed"),
  TEST_HOOD_LEVEL("TestHood/level"),
  TEST_HOOD_POSITION("TestHood/position"),

  // Turret
  TURRET_HW_CONFIGURED("Turret/Turret - hardware_configured"),
  TURRET_HAVE_ZERO("Turret/HaveZero"),
  TURRET_LINED_UP("Turret/LinedUp"),
  TURRET_POSITION_DEG("Turret/Position deg"),
  TURRET_POSITION_RAD("Turret/Position rad"),
  TURRET_TARGET("Turret/Target"),
  TURRET_TARGET_VELOCITY("Turret/Target Velocity"),
  TURRET_VELOCITY_RAD_S("Turret/Velocity (Rad_s))"),
  TURRET_TORQUE("Turret/Torque"),
  TELEOP_TURRET_TARGET_ANGLE("TeleopTurret/targetAngle"),
  TRACK_TURRET_POSITION("Turret/position"),
  TRACK_TURRET_TARGET_POSITION("Turret/targetPosition"),

  // Kicker
  KICKER_HW_CONFIGURED("Kicker/Kicker - hardware_configured"),
  KICKER_VELOCITY("Kicker/Velocity(mps)"),
  KICKER_TARGET_VELOCITY("Kicker/TargetVelocity(mps)"),
  KICKER_CURRENT("Kicker/Current(A)"),

  // Spindexer
  SPINDEXER_HW_CONFIGURED("Spindexer/Spindexer - hardware_configured"),
  SPINDEXER_VELOCITY("Spindexer/Velocity(mps)"),
  SPINDEXER_TARGET_VELOCITY("Spindexer/TargetVelocity(mps)"),
  SPINDEXER_CURRENT("Spindexer/Current(A)"),

  // Climber
  CLIMBER_HW_CONFIGURED("Climber/Climber - hardware_configured"),
  CLIMBER_COMMANDED_VELOCITY("Climber/CommandedVelocity"),
  CLIMBER_COMMANDED_POSITION("Climber/CommandedPosition"),
  CLIMBER_TARGET_POSITION("Climber/TargetPostion"),
  CLIMBER_LAST_COMMAND("Climber/LastCommand"),
  CLIMBER_VELOCITY("Climber/Velocity(mps)"),
  CLIMBER_TARGET_VELOCITY("Climber/TargetVelocity(mps)"),
  CLIMBER_LOAD("Climber/Load(A)"),
  CLIMBER_POSITION("Climber/Position"),

  // Flywheel
  FLYWHEEL_HW_CONFIGURED("Flywheel/Flywheel - hardware_configured"),
  FLYWHEEL_ANGULAR_VELOCITY("Flywheel/AngularVelocity (RPM)"),
  FLYWHEEL_TARGET_VELOCITY("Flywheel/TargetVelocity (mps)"),
  FLYWHEEL_VELOCITY("Flywheel/Velocity (mps)"),
  FLYWHEEL_CURRENT("Flywheel/Current(A)"),
  FLYWHEEL_FOLLOWER_CURRENT("Flywheel/FollowerCurrent(A)"),

  // Intake
  INTAKE_HW_CONFIGURED("Intake/Intake - hardware_configured"),
  INTAKE_POSITION("Intake/Position(rad)"),
  INTAKE_TARGET_POSITION("Intake/TargetPosition(rad)"),
  INTAKE_TORQUE("Intake/Torque(Nm)"),

  // Hood
  HOOD_HW_CONFIGURED("Hood/Hood - hardware_configured"),
  HOOD_ANGLE("Hood/Angle"),
  HOOD_TARGET("Hood/Target"),
  HOOD_TORQUE("Hood/Torque"),

  // Collector
  COLLECTOR_HW_CONFIGURED("Collector/Collector - hardware_configured"),
  COLLECTOR_VELOCITY("Collector/Velocity(mps)"),
  COLLECTOR_TARGET_VELOCITY("Collector/TargetVelocity(mps)"),

  // Localizer
  LOCALIZER_PS("Localizer/PS"),
  LOCALIZER_POSE_X("Localizer/Pose(x)"),
  LOCALIZER_POSE_Y("Localizer/Pose(y)"),
  LOCALIZER_POSE_Q("Localizer/Pose(q)"),
  LOCALIZER_VEL_X("Localizer/Vel(x)"),
  LOCALIZER_VEL_Y("Localizer/Vel(y)"),
  LOCALIZER_VEL_Q("Localizer/Vel(q)"),
  LOCALIZER_MC("Localizer/MC"),

  // TargetFinder
  TARGET_FINDER_TURRET_ANGLE("TargetFinder/Turret Angle"),
  TARGET_FINDER_TURRET_RANGE("TargetFinder/Turret Range"),

  // AprilTag Finder
  APRIL_TAG_FINDER_HAS_TAGS("AprilTagFinder/HasAprilTags"),
  APRIL_TAG_FINDER_USING_TURRET_CAM("AprilTagFinder/UsingTurretCam"),

  // LaserCan
  LASER_CAN_IS_VALID("LaserCan/is_valid"),
  LASER_CAN_HW_CONFIGURED("LaserCan/LaserCAN - hardware_configured"),
  LASER_CAN_DISTANCE("LaserCan/distance(m)"),

  // FieldMap
  FIELD_MAP_TARGET_TAG_ID("FieldMap/TargetTagID"),
  FIELD_MAP_TARGET_POSE_X("FieldMap/TargetPoseX"),
  FIELD_MAP_TARGET_POSE_Y("FieldMap/TargetPoseY"),
  FIELD_MAP_TARGET_POSE_THETA("FieldMap/TargetPoseTheta"),

  // Field Display
  FIELD_DISPLAY_FIELD("Field"),

  // OI
  OI_BALLISTIC_SHOT("OI/BallisticShot"),

  // Zone
  ZONE_ZONE("Zone/Zone"),

  // Auto Event
  AUTO_EVENT("Auto Event");

  private final String key;

  DashboardNames(String key) {
    this.key = key;
  }

  public String getKey() {
    return key;
  }
}
