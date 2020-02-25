package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robots.RobotMap;

public class DriveTrain {

    private TalonFX leftDrive;
    private TalonFX leftDrive2;

    private TalonFX rightDrive;
    private TalonFX rightDrive2;

    private PigeonIMU pigeon;

    public DriveTrain() {

        leftDrive = RobotMap.leftDrive;
        leftDrive2 = RobotMap.leftDrive2;

        rightDrive = RobotMap.rightDrive;
        rightDrive2 = RobotMap.rightDrive2;

        pigeon = RobotMap.pigeon;

        leftDrive.setInverted(true);
        leftDrive2.follow(leftDrive);
        leftDrive2.setInverted(InvertType.FollowMaster);

        rightDrive.setInverted(true);
        rightDrive2.follow(rightDrive);
        rightDrive2.setInverted(InvertType.FollowMaster);

        leftDrive.setSensorPhase(true);
        rightDrive.setSensorPhase(false);

        leftDrive.setNeutralMode(NeutralMode.Coast);
        rightDrive.setNeutralMode(NeutralMode.Coast);

        leftDrive.set(ControlMode.PercentOutput, 0);
        rightDrive.set(ControlMode.PercentOutput, 0);

        leftDrive.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice());
        leftDrive.selectProfileSlot(0, 1);
        rightDrive.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice());
        rightDrive.selectProfileSlot(0, 1);

    }

    public void setNeutralMode(NeutralMode nm) {

        leftDrive.setNeutralMode(nm);
        leftDrive2.setNeutralMode(nm);
        rightDrive.setNeutralMode(nm);
        rightDrive2.setNeutralMode(nm);

    }

    public void SmartDashboard() {

        SmartDashboard.putNumber("Left Velocity", leftDrive.getSelectedSensorVelocity(0));
        SmartDashboard.putNumber("Left Position", leftDrive.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("Left Target Position", leftDrive.getClosedLoopTarget(0));

        SmartDashboard.putNumber("Right Velocity", rightDrive.getSelectedSensorVelocity(0));
        SmartDashboard.putNumber("Right Position", rightDrive.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("Right Target Position", rightDrive.getClosedLoopTarget(0));

        SmartDashboard.putNumber("Fused Heading", getHeading());

    }

    public void driveDirect(double leftThrottle, double rightThrottle) {

        leftDrive.set(ControlMode.PercentOutput, leftThrottle);
        rightDrive.set(ControlMode.PercentOutput, rightThrottle);

    }

    public void arcade(double throttle, double turn) {

        driveDirect(throttle+turn, throttle-turn);

    }

    public void RampRate(double Seconds) {

        leftDrive.configClosedloopRamp(Seconds);
        rightDrive.configClosedloopRamp(Seconds);

    }

    public void setMagic(int cruiseVelocity, int acceleration) {

        leftDrive.configMotionCruiseVelocity(cruiseVelocity);
        leftDrive.configMotionAcceleration(acceleration);

        rightDrive.configMotionCruiseVelocity(cruiseVelocity);
        rightDrive.configMotionAcceleration(acceleration);

    }

    public void resetSensors(int kTimeoutMS) {

        leftDrive.setSelectedSensorPosition(0, 0, kTimeoutMS);
        rightDrive.setSelectedSensorPosition(0, 0, kTimeoutMS);

    }

    public double distanceToRotations(double inches) {

        return (5);

    }

    public void setPosLeft(double nativeUnits) {
    
        leftDrive.set(ControlMode.MotionMagic, nativeUnits);

    }

    public void setPosRight(double nativeUnits) {

        rightDrive.set(ControlMode.MotionMagic, nativeUnits);

    }

    public double getLeftPos() {

        return leftDrive.getSelectedSensorPosition(0);
    }

    public double getRightPos() {

        return rightDrive.getSelectedSensorPosition(0);

    }

    public double getHeading() {

        return pigeon.getFusedHeading();

    }

    public void resetHeading() {

        pigeon.setFusedHeading(0);

    }

}