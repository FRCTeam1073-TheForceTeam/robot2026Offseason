// MAPDISPLAY: gathers and sends data for the map display, accesses drivetrain and localizer
// 

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose3d;

public class MapDisplay extends SubsystemBase{

    private final Field2d field = new Field2d();
    private Drivetrain driveTrain;
    private Localizer localizer;
    private FieldMap fieldMap;
    private Pose3d robotPose;
        
    public MapDisplay(Drivetrain driveTrain, Localizer localizer, FieldMap fieldMap)
    {
        this.driveTrain = driveTrain;
        this.localizer = localizer;
        this.fieldMap = fieldMap;
   
    }
    public String aprilTagAssignments(int tagID)
    {
        HashMap<Integer,String> tagNames = new HashMap<Integer, String>();
        tagNames.put(11,"Far Left");
        tagNames.put(10,"Far Center");
        tagNames.put(9, "Far Right");
        tagNames.put(5,"Close Red Reef");
        tagNames.put(2, "Right Coral Station");
        tagNames.put(1,"Left Coral Station");
        tagNames.put(16,"Red Processor");
        tagNames.put(15,"Far Red Reef");
        tagNames.put(20,"Far Left");
        tagNames.put(21,"Far Center");
        tagNames.put(22,"Far Right");
        tagNames.put(19,"Close Left");
        tagNames.put(18,"Close Center");
        tagNames.put(17,"Close Right");
        tagNames.put(14,"Close Blue Reef");
        tagNames.put(4, "Far Blue Reef");
        tagNames.put(3,"Blue Processor");
        tagNames.put(12,"Right Coral Station");
        tagNames.put(13,"Left Coral Station");
        tagNames.put(6,"Close Left");
        tagNames.put(7,"Close Center");
        tagNames.put(8,"Close Right");
        
        return tagNames.get(tagID);

    }


    @Override
    public void periodic()
    { 
        //field.setRobotPose(driveTrain.getOdometry());
        field.setRobotPose(localizer.getPose());
        SmartDashboard.putData("Field", field); //the widget for this is the dropdown named "field"
        //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/smartdashboard/Field2d.html
        //use above to link to look into two robot poses: 1 odometry, 1 localize
    }
}
