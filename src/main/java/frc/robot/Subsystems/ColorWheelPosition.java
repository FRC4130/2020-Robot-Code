package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Robots.RobotMap;

public class ColorWheelPosition {

    public DoubleSolenoid intakePosition;
    public final Value Stored = Value.kReverse;
    public final Value Sucking = Value.kForward;

    public ColorWheelPosition() {

        intakePosition = RobotMap.intakeLift;
        intakePosition.set(Stored);
        
    }

    public void set(Value vl) {

        intakePosition.set(vl);

    }

    public void disable() {

        intakePosition.set(Value.kOff);
        
    }


}