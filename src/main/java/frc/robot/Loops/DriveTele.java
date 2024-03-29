package frc.robot.Loops;

import com.ctre.phoenix.ILoopable;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Robots.RobotMap;
import frc.robot.Robots.Subsystems;
import frc.robot.Subsystems.DriveTrain;

public class DriveTele implements ILoopable{

    DriveTrain _drive;
    Joystick _joystick;

    public boolean _turn;


    public DriveTele() {

        _drive = Subsystems.driveTrain;
        _joystick = RobotMap.driverJoystick;

        _turn = RobotMap.turn;

    }

    public void onStart() {

        System.out.println("Drive Tele Controls are Starting");

        _drive.setNeutralMode(NeutralMode.Coast);

    }

     public void onLoop() {

        _drive.setNeutralMode(NeutralMode.Brake);
        _drive.RampRate(3);
        _drive.SmartDashboard();

        //20% Speed
        if(_joystick.getRawButton(5)) {

              _drive.driveDirect(_joystick.getRawAxis(1)*-.20, _joystick.getRawAxis(5)*-.20);

          }

          //Turbo VERY DANGEROUS
          else if(_joystick.getRawButton(6)) {

             _drive.driveDirect(_joystick.getRawAxis(1)*-1, _joystick.getRawAxis(5)*-1);

          }
          
          //Normal 60%
          else {

            _drive.driveDirect(_joystick.getRawAxis(1)*-.60, _joystick.getRawAxis(5)*-.60);

          }

         if(_joystick.getRawButton(13)) {

             _drive.resetSensors(20);

         }

         if(_joystick.getRawButton(8)) {

            _drive.setPos(120000);
            _drive.setHeading(72, true);
            _drive.setPos(70000);
         }

    }

    public boolean isDone() {
        return false;

    }

    public void onStop() {

        _drive.driveDirect(0, 0);
        _drive.resetSensors(20);
        _drive.setNeutralMode(NeutralMode.Brake);

    }

}