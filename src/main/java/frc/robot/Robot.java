/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.schedulers.ConcurrentScheduler;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robots.Loops;
import frc.robot.Robots.RobotMap;
import frc.robot.Robots.Subsystems;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.Index;
import frc.robot.Subsystems.IntakePosition;



public class Robot extends TimedRobot {
  ConcurrentScheduler teleop;
  String[] pos = {"Default Reverse", "Defualt Forward", "Left Auton"};
  String gameData;
  DriveTrain x_drive;
  Index x_index;
  IntakePosition x_intakePosition;
  TalonFX x_shootDrive;
  TalonFX x_shootDrive2;
  TalonSRX x_turret;
  PigeonIMU x_pidgey;
  TalonSRX x_pidgeyTalon;
  TalonFX x_leftDrive;
  TalonFX x_rightDrive;
  Joystick x_Joystick;
  double kPgain = 0.03; 				      // percent throttle per degree of error */
	double kDgain = 0.0004; 			      // percent throttle per angular velocity dps */
	double kMaxCorrectionRatio = 0.30;	// cap corrective turning throttle to 30 percent of forward throttle
	double targetAngle = 0;             //  holds the current angle to servo to 
  final int kTimeoutMs = 30;
  int posi = 0;
  Timer matchtimer = new Timer();


  @Override
  public void robotInit() {
    RobotMap.Init();
    Subsystems.Init();
    x_drive = Subsystems.driveTrain;
    x_index = Subsystems.index;
    x_intakePosition = Subsystems.intakePosition;
    x_turret = RobotMap.turret;
    x_shootDrive = RobotMap.Shooter1;
    x_shootDrive2 = RobotMap.Shooter2;
    x_shootDrive2.follow(x_shootDrive);
    x_shootDrive.setInverted(false);
    x_shootDrive2.setInverted(InvertType.OpposeMaster);
    x_shootDrive.setNeutralMode(NeutralMode.Coast);
    x_shootDrive2.setNeutralMode(NeutralMode.Coast);
    x_leftDrive = RobotMap.leftDrive;
    x_rightDrive = RobotMap.rightDrive;
    x_pidgeyTalon = new TalonSRX(RobotMap.kTurretID);
    x_Joystick = RobotMap.driverJoystick;
  }



  @Override
  public void autonomousInit() {
    matchtimer.start();
   }



  @Override
  public void autonomousPeriodic() {

    switch(posi) {

    case 0:
          
          autoTrackingStart();
          autoDriveReverseDefault(6);
          break;

    case 1:
          if(matchtimer.get() > 3 && matchtimer.get() < 7) {
          autoTrackingStart();
          }
          else if (matchtimer.get() > 8 && matchtimer.get() < 10 ) {
          autoDrive();
          }
          else {
            autoMechStop();
            x_drive.driveDirect(0, 0);
          }
          break;

    case 2: 
          if(matchtimer.get() < 4) {
            autoTrackingStart();

          }
          else if (matchtimer.get() < 10) {
            x_intakePosition.set(x_intakePosition.Sucking);
            x_index.runIndex();
            autoDrive();            

          }
          else if(matchtimer.get() < 14) {
            x_drive.driveDirect(0, 0);
            autoTrackingStart2();

          }
          else {
          autoMechStop();
          }
          break;


    /* Not Working Left Auton*/
    // case 3:
    //       autoTrackingStart(4);
    //       x_intakePosition.set(x_intakePosition.Sucking);
    //       x_index.runIndex();
    //       autoDrive(8);
    //       autoTrackingStart2(4);
    //       autoMechStop();
    //       break;

     }
  }

  @Override
  public void disabledPeriodic() {

    if(RobotMap.driverJoystick.getRawButtonPressed(11)) {
      posi++;
    }
  }



  @Override
  public void robotPeriodic() {
    x_pidgey = new PigeonIMU(x_pidgeyTalon);
    final int kTimeoutMs = 30;
    // x_pidgey.setFusedHeading(0.0, kTimeoutMs);
    SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("Match Time", DriverStation.getInstance().getMatchTime());
    SmartDashboard.putString("Auton", pos[posi]);
    gameData = DriverStation.getInstance().getGameSpecificMessage().toLowerCase();
    SmartDashboard.putString("Color Wheel", gameData);
  }



  @Override
  public void teleopInit() {
    teleop = new ConcurrentScheduler();
    Loops.sTeleop(teleop);
    teleop.startAll();
  }



  @Override
  public void teleopPeriodic() {
    teleop.process();
  }



  @Override
  public void testPeriodic() {
  }



  public void autoTrackingStart() {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1); //LED ON
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
      final double STEER_K = 0.030;
      double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
      double steer_cmd = tx * STEER_K;
      double m_LimelightSteerCommand = steer_cmd;
      double area = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
      double targetVeloity = Math.abs((-1459.5* area)+10139);
      SmartDashboard.putNumber("Target Velocity", targetVeloity);
      x_turret.set(ControlMode.PercentOutput, m_LimelightSteerCommand);
      x_shootDrive.set(ControlMode.Velocity,  targetVeloity);
      if(x_shootDrive.getSelectedSensorVelocity() >  Math.abs(targetVeloity - 50)) {
        x_index.shootMode();
      }
  }



  public void autoTrackingStart2() {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(2); //LED ON
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
      final double STEER_K = 0.028;
      double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
      double steer_cmd = tx * STEER_K;
      double m_LimelightSteerCommand = steer_cmd;
      double area = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
      double targetVeloity = Math.abs((-1559.5* area)+10139);  //(-1549.5*area)+10139
      SmartDashboard.putNumber("Target Velocity", targetVeloity);
      x_turret.set(ControlMode.PercentOutput, m_LimelightSteerCommand);
      x_shootDrive.set(ControlMode.Velocity,  targetVeloity);
      if(x_shootDrive.getSelectedSensorVelocity() >  Math.abs(targetVeloity - 50)) {
        x_index.shootMode();
      }
  }

  public void autoDrive() {


      x_drive.driveDirect(.20, .21);

  }

  public void autoDriveReverseDefault(double timeout) {

    Timer timerx = new Timer();
    timerx.start();
    do {

      x_drive.driveDirect(-.20, -.21);
    }
    while (timerx.get() <= timeout);

  }

  public void autoDriveForwardDefault(double timeout) {

    Timer timerx = new Timer();
    timerx.start();
    do {

      x_drive.driveDirect(.20, .21);
    }
    while (timerx.get() <= timeout);

  }

  public void autoMechStop(){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    x_turret.set(ControlMode.PercentOutput, 0);
    x_shootDrive.set(ControlMode.PercentOutput, 0);
    x_index.stopIndex();
  }



  // public void autoDriveStraight(double timeout){
  //   Timer timerx = new Timer();
  //   timerx.start();
  //   do {
  //     PigeonIMU.GeneralStatus genStatus = new PigeonIMU.GeneralStatus();
  //     PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
  //     double [] xyz_dps = new double [3];
  //     x_pidgey.getGeneralStatus(genStatus);
  //     x_pidgey.getRawGyro(xyz_dps);
  //     x_pidgey.getFusedHeading(fusionStatus);
  //     double currentAngle = fusionStatus.heading;
  //     boolean angleIsGood = (x_pidgey.getState() == PigeonIMU.PigeonState.Ready) ? true : false;
  //     double currentAngularRate = xyz_dps[1];// Array to fill with x[0], y[1], and z[2] data in degrees per second.
  //     SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
  //     SmartDashboard.putNumber("Pigeon Fused Heading", x_pidgey.getFusedHeading());
  //     SmartDashboard.putNumber("Pigeon Device ID", x_pidgey.getDeviceID());
  //     SmartDashboard.putNumber("Current Angle", currentAngle);
  //     SmartDashboard.putBoolean("Angle Good?", angleIsGood);
  //     SmartDashboard.putNumber("Current Anglular Rate", currentAngularRate);
  //     double forwardThrottle = .20;
  //     double turnThrottle = (targetAngle - currentAngle) * kPgain - (currentAngularRate) * kDgain;
  //     double left = forwardThrottle - turnThrottle;
  //     double right = forwardThrottle + turnThrottle;
  //     x_leftDrive.set(ControlMode.PercentOutput, left);
  //     x_rightDrive.set(ControlMode.PercentOutput, right);
  //     SmartDashboard.putNumber("Turn Throttle", turnThrottle);
  //     SmartDashboard.putNumber("Forward Throttle", forwardThrottle);
  //     SmartDashboard.putNumber("Target Angle", targetAngle);
  //   }
  //   while (timerx.get() <= timeout);
  // }
}
