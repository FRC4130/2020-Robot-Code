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
import frc.robot.Robots.RobotMap;

public class Turret implements ILoopable{

    TalonSRX turret;
    TalonFX shootDrive;
    TalonFX shootDrive2;

    Joystick _Joystick;

    private final int kTimeoutMS = 30;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    double m_LimelightDriveCommand = 0.0;
    double m_LimelightSteerCommand = 0.0;
    boolean m_LimelightHasValidTarget = false;

    public Turret() {

        turret = RobotMap.turret;
        shootDrive = RobotMap.Shooter1;
        shootDrive2 = RobotMap.Shooter2;

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
      shootDrive.setInverted(true);
      shootDrive2.setInverted(InvertType.OpposeMaster);
      turret.setNeutralMode(NeutralMode.Brake);
      shootDrive.setNeutralMode(NeutralMode.Coast);
      shootDrive2.setNeutralMode(NeutralMode.Coast);

      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    }

    public void onLoop() {
    
        LimelightTracking();

        if (_Joystick.getRawButton(2)) {
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
            if (m_LimelightHasValidTarget) {
                turret.set(ControlMode.PercentOutput, m_LimelightSteerCommand);
                shootDrive.set(ControlMode.PercentOutput, .7);
            }    
            else {
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
                shootDrive.set(ControlMode.PercentOutput, 0);
            }
        }    
        else if (_Joystick.getRawButton(5)){
            turret.set(ControlMode.PercentOutput, -.2);
        }
        else if (_Joystick.getRawButton(6)){
            turret.set(ControlMode.PercentOutput, .2);
        }
        else if (_Joystick.getRawButton(13)){
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
        }  
        else{
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);

            turret.set(ControlMode.PercentOutput, 0);
            shootDrive.set(ControlMode.PercentOutput, 0);
        }
    }

    public void LimelightTracking() {
        // These numbers must be tuned...
        final double STEER_K = 0.030; // How hard to turn toward the target
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
        double steer_cmd = tx * STEER_K;
        m_LimelightSteerCommand = steer_cmd;
        // Try to drive forward until the target area reaches our desired area
        double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;
        // Don't let the robot drive too fast into the target
        if (drive_cmd > MAX_DRIVE) {
        drive_cmd = MAX_DRIVE;
        }
        m_LimelightDriveCommand = drive_cmd;
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