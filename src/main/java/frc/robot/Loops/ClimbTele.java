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

        _climb.setNeutralModeLeft(NeutralMode.Brake);
        _climb.setNeutralModeRight(NeutralMode.Brake);

    }

    public void onLoop() {

        _climb.Drive(_joystick.getRawAxis(5));
        _climb.Drive2(_joystick.getRawAxis(1));
        
    }

    
    public boolean isDone() {
        return false;

    }

    public void onStop() {

        _climb.setNeutralModeLeft(NeutralMode.Brake);
        _climb.setNeutralModeRight(NeutralMode.Brake);
        _climb.Drive(0);

    }
    
}