package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Robots.RobotMap;

public class ClimbRelease {

    public DoubleSolenoid climbSolenoid;
    public final Value Secured = Value.kReverse;
    public final Value Released = Value.kForward;

    public ClimbRelease() {

        climbSolenoid = RobotMap.climberRelease;
        climbSolenoid.set(Secured);
        
    }

    public void set(Value vl) {

        climbSolenoid.set(vl);

    }

    public void disable() {

        climbSolenoid.set(Value.kOff);
        
    }


}