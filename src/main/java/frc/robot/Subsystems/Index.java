package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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

    private VictorSPX intake;

    public Index() {


        index1 = RobotMap.Index1;
        index2 = RobotMap.Index2;
        index3 = RobotMap.Index3;
        index4 = RobotMap.Index4;
        index5 = RobotMap.Index5;

        intake = RobotMap.Intake;

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

        SmartDashboard.putNumber("Index 1 Sensors", index1.isRevLimitSwitchClosed());
        SmartDashboard.putNumber("Index 2 Sensors", index2.isRevLimitSwitchClosed());
        SmartDashboard.putNumber("Index 3 Sensors", index3.isRevLimitSwitchClosed());
        SmartDashboard.putNumber("Index 4 Sensors", index4.isRevLimitSwitchClosed());
        SmartDashboard.putNumber("Index 5 Sensors", index5.isRevLimitSwitchClosed());
    }

    public void setIndexNeutralMode(NeutralMode nm) {

        index1.setNeutralMode(nm);
        index2.setNeutralMode(nm);
        index3.setNeutralMode(nm);
        index4.setNeutralMode(nm);
        index5.setNeutralMode(nm);

    }

    public void runIndex() {

        if(index1.isRevLimitSwitchClosed() == 0) {

            index1.set(ControlMode.PercentOutput, 1);
    
           }
    
           else if(index2.isRevLimitSwitchClosed() == 0) {
    
            index1.set(ControlMode.PercentOutput, 1);
    
           }
    
           else {
            index1.set(ControlMode.PercentOutput, 0);
    
           }
    
           if(index2.isRevLimitSwitchClosed() == 0) {
    
            index2.set(ControlMode.PercentOutput, 1);
    
           }
    
           else if(index3.isRevLimitSwitchClosed() == 0) {
    
            index2.set(ControlMode.PercentOutput, 1);
    
           }
    
           else {
            index2.set(ControlMode.PercentOutput, 0);
    
           }
    
           if(index3.isRevLimitSwitchClosed() == 0) {
    
            index3.set(ControlMode.PercentOutput, 1);
    
           }
    
           else if(index4.isRevLimitSwitchClosed() == 0) {
    
            index3.set(ControlMode.PercentOutput, 1);
    
           }
    
           else {
    
            index3.set(ControlMode.PercentOutput, 0);
    
           }
    
           if(index4.isRevLimitSwitchClosed() == 0) {
    
            index4.set(ControlMode.PercentOutput, 1);
    
           }
    
           else if (index5.isRevLimitSwitchClosed() == 0) {
    
            index4.set(ControlMode.PercentOutput, 1);
    
           }
    
           else {
    
            index4.set(ControlMode.PercentOutput, 0);
    
           }
    
           if (index5.isRevLimitSwitchClosed() == 0) {
    
            index5.set(ControlMode.PercentOutput, .50);
    
           }
    
           else {
    
            index5.set(ControlMode.PercentOutput, 0);
    
           }
    
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

    public void shootMode() {

        if(index1.isRevLimitSwitchClosed() == 0) {

            index1.set(ControlMode.PercentOutput, 1);
    
           }
    
           else if(index2.isRevLimitSwitchClosed() == 0) {
    
            index1.set(ControlMode.PercentOutput, 1);
    
           }
    
           else {
            index1.set(ControlMode.PercentOutput, 0);
    
           }
    
           if(index2.isRevLimitSwitchClosed() == 0) {
    
            index2.set(ControlMode.PercentOutput, 1);
    
           }
    
           else if(index3.isRevLimitSwitchClosed() == 0) {
    
            index2.set(ControlMode.PercentOutput, 1);
    
           }
    
           else {
            index2.set(ControlMode.PercentOutput, 0);
    
           }
    
           if(index3.isRevLimitSwitchClosed() == 0) {
    
            index3.set(ControlMode.PercentOutput, 1);
    
           }
    
           else if(index4.isRevLimitSwitchClosed() == 0) {
    
            index3.set(ControlMode.PercentOutput, 1);
    
           }
    
           else {
    
            index3.set(ControlMode.PercentOutput, 0);
    
           }
    
           if(index4.isRevLimitSwitchClosed() == 0) {
    
            index4.set(ControlMode.PercentOutput, 1);
    
           }
    
           else if (index5.isRevLimitSwitchClosed() == 0) {
    
            index4.set(ControlMode.PercentOutput, 1);
    
           }
    
           else {
    
            index4.set(ControlMode.PercentOutput, 0);
    
           }
    
           if (index5.isRevLimitSwitchClosed() == 0) {
    
            index5.set(ControlMode.PercentOutput, 1);
    
           }
    
           else {
    
            
            index5.set(ControlMode.PercentOutput, 1);
    
           }
    }

    public void PurgeMode() {
        index1.set(ControlMode.PercentOutput, .70);
        index2.set(ControlMode.PercentOutput, .70);
        index3.set(ControlMode.PercentOutput, .70);
        index4.set(ControlMode.PercentOutput, .70);
        index5.set(ControlMode.PercentOutput, .70);
    }

}  