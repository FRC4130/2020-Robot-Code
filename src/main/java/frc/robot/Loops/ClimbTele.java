package frc.robot.Loops;

import com.ctre.phoenix.ILoopable;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Robots.RobotMap;
import frc.robot.Robots.Subsystems;
import frc.robot.Subsystems.Climb;
import frc.robot.Subsystems.ClimbRelease;

public class ClimbTele implements ILoopable{

    Climb _climb;
    Joystick _joystick;
    ClimbRelease _climbSolenoid;

    public ClimbTele() {

        _climb = Subsystems.climb;
        _joystick = RobotMap.operatorJoystick;
        _climbSolenoid = Subsystems.climbRelease;

    }

    public void onStart() {

        System.out.println("ClimbTele Controls has Started!");

        _climb.setNeutralModeLeft(NeutralMode.Coast);
        _climb.setNeutralModeRight(NeutralMode.Coast);
        _climbSolenoid.set(_climbSolenoid.Secured);

    }

    public void onLoop() {

        if(_joystick.getRawButton(9) && _joystick.getRawButton(10)) {
            _climbSolenoid.set(_climbSolenoid.Released);

            _climb.DriveRight(_joystick.getRawAxis(1));
            _climb.DriveLeft(_joystick.getRawAxis(1)*.95);

        }

        else {
            _climbSolenoid.set(_climbSolenoid.Secured);

        }
        
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