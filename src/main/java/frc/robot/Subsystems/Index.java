package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robots.RobotMap;


public class Index {

    private TalonSRX index1;
    private TalonSRX index2;
    private TalonSRX index3;
    private TalonSRX index4;
    private TalonSRX index5;

    private TalonFX shooter1;
    private TalonFX shooter2;

    private VictorSPX intake;

    public Index() {


        index1 = RobotMap.Index1;
        index2 = RobotMap.Index2;
        index3 = RobotMap.Index3;
        index4 = RobotMap.Index4;
        index5 = RobotMap.Index5;

        shooter1 = RobotMap.Shooter1;
        shooter2 = RobotMap.Shooter2;

        intake = RobotMap.Intake;

        shooter1.setInverted(true);
        shooter2.follow(shooter1);
        shooter2.setInverted(InvertType.OpposeMaster);

        shooter1.setNeutralMode(NeutralMode.Coast);

        index1.setNeutralMode(NeutralMode.Brake);
        index2.setNeutralMode(NeutralMode.Brake);
        index3.setNeutralMode(NeutralMode.Brake);
        index4.setNeutralMode(NeutralMode.Brake);
        index5.setNeutralMode(NeutralMode.Brake);

        index1.setInverted(false);
        index2.setInverted(true);
        index3.setInverted(true);
        index4.setInverted(true);
        index5.setInverted(true);

    }

    public void SmartDashboard() {

        SmartDashboard.putNumber("Index 1 Sensor Value", index1.getSensorCollection().getAnalogIn());
        SmartDashboard.putNumber("Index 2 Sensor Value", index2.getSensorCollection().getAnalogIn());
        SmartDashboard.putNumber("Index 3 Sensor Value", index3.getSensorCollection().getAnalogIn());
        SmartDashboard.putNumber("Index 4 Sensor Value", index4.getSensorCollection().getAnalogIn());
        SmartDashboard.putNumber("Index 5 Sensor Value", index5.getSensorCollection().getAnalogIn());
    }

    public void setIndexNeutralMode(NeutralMode nm) {

        index1.setNeutralMode(nm);
        index2.setNeutralMode(nm);
        index3.setNeutralMode(nm);
        index4.setNeutralMode(nm);
        index5.setNeutralMode(nm);

    }

    public void runIndex() {

        if(index1.getSensorCollection().getAnalogInRaw() > 90) {

            index1.set(ControlMode.PercentOutput, .50);
    
           }
    
           else if(index2.getSensorCollection().getAnalogInRaw() > 20) {
    
            index1.set(ControlMode.PercentOutput, .50);
    
           }
    
           else {
            index1.set(ControlMode.PercentOutput, 0);
    
           }
    
           if(index2.getSensorCollection().getAnalogInRaw() > 20) {
    
            index2.set(ControlMode.PercentOutput, .50);
    
           }
    
           else if(index3.getSensorCollection().getAnalogInRaw() > 20) {
    
            index2.set(ControlMode.PercentOutput, .50);
    
           }
    
           else {
            index2.set(ControlMode.PercentOutput, 0);
    
           }
    
           if(index3.getSensorCollection().getAnalogInRaw() > 90) {
    
            index3.set(ControlMode.PercentOutput, .50);
    
           }
    
           else if(index4.getSensorCollection().getAnalogInRaw() > 90) {
    
            index3.set(ControlMode.PercentOutput, .50);
    
           }
    
           else {
    
            index3.set(ControlMode.PercentOutput, 0);
    
           }
    
           if(index4.getSensorCollection().getAnalogInRaw() > 90) {
    
            index4.set(ControlMode.PercentOutput, .50);
    
           }
    
           else if (index5.getSensorCollection().getAnalogInRaw() > 90) {
    
            index4.set(ControlMode.PercentOutput, .50);
    
           }
    
           else {
    
            index4.set(ControlMode.PercentOutput, 0);
    
           }
    
           if (index5.getSensorCollection().getAnalogInRaw() > 90) {
    
            index5.set(ControlMode.PercentOutput, .50);
    
           }
    
           else {
    
            index5.set(ControlMode.PercentOutput, 0);
    
           }
    
    }

    public void Shoot() {
        

    }

    public void Intake(double intakePower) {

        intake.set(ControlMode.PercentOutput, intakePower);

    }

    public void stopIndex() {

        index1.set(ControlMode.PercentOutput, 0);
        index2.set(ControlMode.PercentOutput, 0);
        index3.set(ControlMode.PercentOutput, 0);
        index4.set(ControlMode.PercentOutput, 0);
        index5.set(ControlMode.PercentOutput, 0);

    }

}  