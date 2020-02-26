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
        // climbRight.follow(climbLeft);
        climbRight.setInverted(false);

        climbLeft.set(ControlMode.PercentOutput, 0);

    }

    public void setNeutralModeLeft(NeutralMode nm) {

        climbLeft.setNeutralMode(nm);

    }

    public void setNeutralModeRight(NeutralMode nm) {

        climbRight.setNeutralMode(nm);

    }

    public void Drive(double throttle) {

        climbLeft.set(ControlMode.PercentOutput, throttle);

    }

    public void Drive2(double throttle) {

        climbRight.set(ControlMode.PercentOutput, throttle);

    }

}