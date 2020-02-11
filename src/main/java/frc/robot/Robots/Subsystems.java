package frc.robot.Robots;

import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.Index;
import frc.robot.Subsystems.IntakePosition;

public class Subsystems {

    public static DriveTrain driveTrain;
    public static Index index;
    public static IntakePosition intakePosition;


    public static void Init() {

        driveTrain = new DriveTrain();
        index = new Index();
        intakePosition = new IntakePosition();

    }


}