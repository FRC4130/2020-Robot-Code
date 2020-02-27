package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Robots.RobotMap;

public class Climb {

    public TalonFX climbLeft;
    public TalonFX climbRight;
    public Climb() {

        climbLeft = RobotMap.ClimberLeft;
        climbRight = RobotMap.ClimberRight;

        climbLeft.setInverted(true);
        //climbRight.follow(climbLeft);
        climbRight.setInverted(false);

        climbLeft.set(ControlMode.PercentOutput, 0);

        climbLeft.configNominalOutputForward(0);
        climbLeft.configNominalOutputReverse(0);
        climbLeft.configPeakOutputForward(1);
        climbLeft.configPeakOutputReverse(1);

        climbRight.configNominalOutputForward(0);
        climbRight.configNominalOutputReverse(0);
        climbRight.configPeakOutputForward(1);
        climbRight.configPeakOutputReverse(1);
    }

    public void setNeutralModeLeft(NeutralMode nm) {

        climbLeft.setNeutralMode(nm);

    }

    public void setNeutralModeRight(NeutralMode nm) {

        climbRight.setNeutralMode(nm);

    }

    public void DriveLeft(double throttle) {

        climbLeft.set(ControlMode.PercentOutput, throttle);

    }

    public void DriveRight(double throttle) {

        climbRight.set(ControlMode.PercentOutput, throttle);

    }

}