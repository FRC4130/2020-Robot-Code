package frc.robot.Loops;

import com.ctre.phoenix.ILoopable;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Robots.RobotMap;
import frc.robot.Robots.Subsystems;
import frc.robot.Subsystems.Index;
import frc.robot.Subsystems.IntakePosition;

public class IndexTele implements ILoopable{

    Index _index;
    IntakePosition _IntakePosition;
    Joystick _joystick;

    boolean indexMove;

    public IndexTele() {

        _index = Subsystems.index;
        _IntakePosition = Subsystems.intakePosition;
        _joystick = RobotMap.operatorJoystick;

    }

    public void onStart() {

        System.out.println("Index Tele Controls are Starting");

        _index.setIndexNeutralMode(NeutralMode.Brake);
        _IntakePosition.set(_IntakePosition.Stored);

    }

    public void onLoop() {

         if(_joystick.getRawButton(5)) {

             _index.runIndex();
             _index.Intake(-.60);
             _IntakePosition.set(_IntakePosition.Sucking);

         }

         else {

           if (!_joystick.getRawButton(2)){
                _index.stopIndex();
            }
           _index.Intake(0);
           _IntakePosition.set(_IntakePosition.Stored);

         }

         _index.SmartDashboard();

            indexMove = _joystick.getRawButton(5);

        if(_joystick.getRawButton(5)) {

            updateIntakeSolenoid();

        }

    }

    private void updateIntakeSolenoid() {

        if(indexMove) {
            _IntakePosition.set(_IntakePosition.Sucking);

        }

        else {
            _IntakePosition.set(_IntakePosition.Stored);

        }

    }

    public boolean isDone() {
        return false;

    }

    public void onStop() {

        System.out.println("[WARNING] Index Tele has Stopped!");

        _index.stopIndex();
        _index.setIndexNeutralMode(NeutralMode.Coast);
        _IntakePosition.set(_IntakePosition.Stored);


    }


}