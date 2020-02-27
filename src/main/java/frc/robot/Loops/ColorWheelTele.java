package frc.robot.Loops;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Robots.RobotMap;
import frc.robot.Robots.Subsystems;
import frc.robot.Subsystems.ColorWheel;

public class ColorWheelTele {

    ColorWheel _colorWheel;

    Joystick _joystick;

    public ColorWheelTele() {

        _colorWheel = Subsystems.colorWheel;

        _joystick = RobotMap.driverJoystick;

    }

}