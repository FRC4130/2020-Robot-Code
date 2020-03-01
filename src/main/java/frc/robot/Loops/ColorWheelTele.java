package frc.robot.Loops;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Robots.RobotMap;
import frc.robot.Robots.Subsystems;
import frc.robot.Subsystems.ColorWheel;
import frc.robot.Subsystems.ColorWheelPosition;

public class ColorWheelTele {

    ColorWheel _colorWheel;
    ColorWheelPosition _colorPosition;
    Joystick _joystick;

    public ColorWheelTele() {

        _colorWheel = Subsystems.colorWheel;
        _colorPosition = Subsystems.wheelposition;
        _joystick = RobotMap.driverJoystick;

    }

    public void onStart() {

        

    }
    

}