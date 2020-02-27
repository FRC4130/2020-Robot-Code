package frc.robot.Loops;

import com.ctre.phoenix.ILoopable;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Robots.RobotMap;
import frc.robot.Robots.Subsystems;
import frc.robot.Subsystems.Climb;

public class ClimbTele implements ILoopable{

    Climb _climb;
    Joystick _joystick;

    public ClimbTele() {

        _climb = Subsystems.climb;
        _joystick = RobotMap.operatorJoystick;

    }

    public void onStart() {

        System.out.println("ClimbTele Controls has Started!");

        _climb.setNeutralModeLeft(NeutralMode.Coast);
        _climb.setNeutralModeRight(NeutralMode.Coast);

    }

    public void onLoop() {

        _climb.DriveRight(_joystick.getRawAxis(5));

         _climb.DriveLeft(_joystick.getRawAxis(5)*.95);

        
    }

    
    public boolean isDone() {
        return false;

    }

    public void onStop() {

        _climb.setNeutralModeLeft(NeutralMode.Coast);
        _climb.setNeutralModeRight(NeutralMode.Coast);
        _climb.DriveLeft(0);
        _climb.DriveRight(0);

    }
    
}