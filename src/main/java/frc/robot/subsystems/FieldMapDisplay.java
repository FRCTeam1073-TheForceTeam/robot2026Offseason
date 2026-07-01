// FIELDMAPDISPLAY: gathers and sends data for the map display, accesses drivetrain and localizer

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FieldMapDisplay extends SubsystemBase
{
    private final Field2d field = new Field2d();
    private final Drivetrain driveTrain;
    private final Localizer localizer;
    private final FieldMap fieldMap;

    public FieldMapDisplay(Drivetrain driveTrain, Localizer localizer, FieldMap fieldMap)
    {
        this.driveTrain = driveTrain;
        this.localizer = localizer;
        this.fieldMap = fieldMap;
    }

    @Override
    public void periodic()
    {
        field.setRobotPose(localizer.getPose());
        SmartDashboard.putData("Field", field); // the widget for this is the dropdown named "field"
    }
}
