package frc.robot.Subsystems;

import com.ctre.phoenix.ILoopable;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robots.RobotMap;
import frc.robot.Robots.Subsystems;

public class Turret implements ILoopable{

    TalonSRX turret;
    TalonFX shootDrive;
    TalonFX shootDrive2;

    Index _index;

    Joystick _Joystick;

    private final int kTimeoutMS = 30;

    double ActualVelocity = 0;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    double m_LimelightDriveCommand = 0.0;
    double m_LimelightSteerCommand = 0.0;
    boolean m_LimelightHasValidTarget = false;

    double WantedPipeline;
    double ChangedModifier;

    public Turret() {

        turret = RobotMap.turret;
        shootDrive = RobotMap.Shooter1;
        shootDrive2 = RobotMap.Shooter2;

        _index = Subsystems.index;
        _Joystick = RobotMap.operatorJoystick;

    }

    public void onStart() {
      shootDrive.configNominalOutputForward(0.0, kTimeoutMS);
      shootDrive.configNominalOutputReverse(0.0, kTimeoutMS);
      shootDrive.configPeakOutputForward(1.0, kTimeoutMS);
      shootDrive.configPeakOutputReverse(-1.0, kTimeoutMS);
      shootDrive2.configNominalOutputForward(0.0, kTimeoutMS);
      shootDrive2.configNominalOutputReverse(0.0, kTimeoutMS);
      shootDrive2.configPeakOutputForward(1.0, kTimeoutMS);
      shootDrive2.configPeakOutputReverse(-1.0, kTimeoutMS);
      shootDrive2.follow(shootDrive);
      turret.setInverted(false);
      shootDrive.setInverted(false);
      shootDrive2.setInverted(InvertType.OpposeMaster);
      turret.setNeutralMode(NeutralMode.Brake);
      shootDrive.setNeutralMode(NeutralMode.Coast);
      shootDrive2.setNeutralMode(NeutralMode.Coast);

      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    }

    public void onLoop() {

        ActualVelocity = shootDrive.getSelectedSensorVelocity();

        PipelineSwitch();

        LimelightTracking();

        if (_Joystick.getRawButton(2)) {
            // NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);

            PipelineSwitch();
            double area = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
            double targetVeloity = Math.abs((-1459.5* area)+10139);  //(-1549.5*area)+10139
            SmartDashboard.putNumber("Target Velocity", targetVeloity);

            if (m_LimelightHasValidTarget) {
                turret.set(ControlMode.PercentOutput, m_LimelightSteerCommand);
                shootDrive.set(ControlMode.Velocity,  targetVeloity);
                /* 

                    85% = Zone 3
                    60% = Zone 2
                    42% = Zone 1

                */ 

                if(shootDrive.getSelectedSensorVelocity() >  Math.abs(targetVeloity - 50)) {
                    _index.shootMode();

                }
                
                // else {
                //     _index.stopIndex();
                // }

            }    
            else {
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
                shootDrive.set(ControlMode.PercentOutput, 50);
                _index.stopIndex();
            }
        }
        else if (_Joystick.getRawButton(3)) {

                    shootDrive.set(ControlMode.Velocity, 11000);
    
                    if(shootDrive.getSelectedSensorVelocity() > Math.abs(11000-100)) {
                        _index.shootMode();
                    }
                    else {
                        _index.stopIndex();
                    }
    
    
             }
        else if (_Joystick.getRawButton(4)) {
            shootDrive.set(ControlMode.PercentOutput, .40);
            _index.shootMode();
        }
        else if(_Joystick.getRawButton(14)) {
            shootDrive.set(ControlMode.PercentOutput, .23);
            _index.PurgeMode();
        } 
        else if (_Joystick.getRawButton(13)){
            PipelineSwitch();
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
        }  
        else if (_Joystick.getRawButton(11)){
            PipelineSwitch();
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
            turret.set(ControlMode.PercentOutput, m_LimelightSteerCommand);
        } 
        else{
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);

            turret.set(ControlMode.PercentOutput, 0);
            shootDrive.set(ControlMode.PercentOutput, 0);

        }

        if (_Joystick.getRawButton(7)){
            turret.set(ControlMode.PercentOutput, -.17);
        }
        else if (_Joystick.getRawButton(8)){
            turret.set(ControlMode.PercentOutput, .17);
        }
        // else {
        //     turret.set(ControlMode.PercentOutput, 0);
        // }

        SmartDashboard.putNumber("RPM Shooter", Math.abs(((ActualVelocity*600)/2048)*2));
        SmartDashboard.putNumber("TA Value of Limelight", NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0));
    }

    public void LimelightTracking() {
        // These numbers must be tuned...
        final double STEER_K = 0.026; // How hard to turn toward the target
        final double DRIVE_K = 0.40; // How hard to drive fwd toward the target
        final double DESIRED_TARGET_AREA = 1.2; // Area of the target when the robot reaches the wall
        final double MAX_DRIVE = .70; // Speed limit so we don't drive too fast
        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
        // Function to determine autonomous drive and steering settings
        if (tv < 0.5) {
            m_LimelightHasValidTarget = false;
            m_LimelightDriveCommand = 0.0;
            m_LimelightSteerCommand = 0.0;
        return;
        }
        m_LimelightHasValidTarget = true;
        // Start with proportional steering
        double steer_cmd = ((tx * STEER_K) + ChangedModifier); //ZONE 1 = .06
        m_LimelightSteerCommand = steer_cmd;
        // Try to drive forward until the target area reaches our desired area
        double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;
        // Don't let the robot drive too fast into the target
        if (drive_cmd > MAX_DRIVE) {
        drive_cmd = MAX_DRIVE;
        }
        m_LimelightDriveCommand = drive_cmd;
    }

    public void PipelineSwitch() {

        if(_Joystick.getPOV() > -1) {

            if(_Joystick.getPOV() == 90) {
            WantedPipeline = 1; 

            }
            else if(_Joystick.getPOV() == 180) {
            WantedPipeline = 2;
            }
            else if(_Joystick.getPOV() == 270) {
            WantedPipeline = 3;
            }
            else if(_Joystick.getPOV() == 0) {
            WantedPipeline = 4;
            }
        }
        
        if(WantedPipeline == 1) {
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
            ChangedModifier = 0;
        }
        else if(WantedPipeline == 2) {
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(2);
            ChangedModifier = 0;
        }
        else if(WantedPipeline == 3) {
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(3);
            ChangedModifier = 0;
        }
        else if(WantedPipeline == 4) {
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(4);
            ChangedModifier = 0;
        }
        SmartDashboard.putNumber("Pipeline Zone", WantedPipeline);
    }

    public boolean isDone() {
        return false;

    }

    public void onStop() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
        turret.set(ControlMode.PercentOutput, 0);
        shootDrive.set(ControlMode.PercentOutput, 0);
    }
}