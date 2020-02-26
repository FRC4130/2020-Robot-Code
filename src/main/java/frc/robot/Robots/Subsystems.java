package frc.robot.Robots;

import frc.robot.Subsystems.Climb;
import frc.robot.Subsystems.ColorWheel;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.Index;
import frc.robot.Subsystems.IntakePosition;
import frc.robot.Subsystems.Turret;

public class Subsystems {

    public static DriveTrain driveTrain;
    public static Index index;
    public static IntakePosition intakePosition;
    public static Turret turret;
    public static ColorWheel colorWheel;
    public static Climb climb;


    public static void Init() {

        driveTrain = new DriveTrain();
        index = new Index();
        intakePosition = new IntakePosition();
        turret = new Turret();
        colorWheel = new ColorWheel();
        climb = new Climb();

    }


}