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
import frc.robot.Loops.DriveDistance;
import frc.robot.Robots.Loops;
import frc.robot.Robots.RobotMap;
import frc.robot.Robots.Subsystems;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.Index;
import frc.robot.Subsystems.IntakePosition;



public class Robot extends TimedRobot {
  ConcurrentScheduler teleop;
  String[] pos = {"Delayed Forward", "Default Forward", "6 Ball Trench", "6 Ball Trench Middle", "Encoder & Pixy Test", "Barrel Race", "Slalom Path", "Bounce Path"};
  String gameData;
  DriveTrain x_drive;
  DriveDistance x_distance;
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

    if(posi == 4 || posi == 5 || posi == 6 || posi == 7) {

      x_drive.RampRate(3);

    }
    else {
      x_drive.RampRate(0);
    }
    switch(posi) {

    /* --- Delayed Shoot, Drive Forward --- */
    case 0:
          if(matchtimer.get() > 3 && matchtimer.get() < 7) {
          autoTrackingStart();
          }
          else if (matchtimer.get() > 8 && matchtimer.get() < 10 ) {
          autoDriverightTrench();
          }
          else {
            autoMechStop();
            x_drive.driveDirect(0, 0);
          }
          break;

    /* --- Shoot, Drive Forward --- */
    case 1:
          if(matchtimer.get() < 4) {
          autoTrackingStart();
          }
          else if (matchtimer.get() < 6 ) {
          autoDriverightTrench();
          }
          else {
            autoMechStop();
            x_drive.driveDirect(0, 0);
          }
          break;

    /* --- Right Trench (6 Ball) --- */
    case 2: 
          if(matchtimer.get() < 4) {
            autoTrackingStart();

          }
          else if (matchtimer.get() < 7) {
            x_intakePosition.set(x_intakePosition.Sucking);
            x_index.runIndex();
            autoDriverightTrench();            

          }
          else if(matchtimer.get() < 14) {
            x_drive.driveDirect(0, 0);
            autoTrackingStart2();

          }
          else {
          autoMechStop();
          }
          break;

    /* --- Right Trench Middle (6 Ball) --- */
    case 3:
          if(matchtimer.get() < 3) {
            autoTrackingStart();
            x_drive.SmartDashboard();
          }
          else if(matchtimer.get() < 5) {
            x_intakePosition.set(x_intakePosition.Sucking);
            x_index.runIndex();
            autoTracking();
            autoDrive2();
            x_drive.SmartDashboard();
          }
          else if(matchtimer.get() < 7) {
            x_intakePosition.set(x_intakePosition.Sucking);
            x_index.runIndex();
            autoTracking();
            autoDrive();
            x_drive.SmartDashboard();
          }

          else if(matchtimer.get() < 15) {
            x_drive.driveDirect(0, 0);
            autoTrackingStartMiddle2();
            x_drive.SmartDashboard();
          }
          else {
            autoMechStop();
          }
          break;

          /* --- 1 inch = 1,0000 encoder pulses --- */
          /* ---If pigeon is negative means we turn right ---  */
    /** --- Encoder & Pixy Test --- */
    case 4:
     x_drive.setPos(120000);
     x_drive.resetSensors(20);
     x_drive.setHeading(45, true);
     x_drive.resetSensors(20);
     x_drive.setPos(70000);
     break;
      
    /* --- Barrel Race Path --- */
    case 5:
    //start drive
     x_drive.setPos(120000); //forward 120 inches
     x_drive.resetSensors(20);
     //doing first circle
     x_drive.setHeading(-90, true); //right 90
     x_drive.resetSensors(20);
     x_drive.setPos(30000); //forward 30 inches
     x_drive.resetSensors(20);
     x_drive.setHeading(-90, true); //right 90
     x_drive.resetSensors(20);
     x_drive.setPos(30000); //forward 30 inches
     x_drive.resetSensors(20);
     x_drive.setHeading(-90, true); //right 90
     x_drive.resetSensors(20);
     x_drive.setPos(30000); //forward 30 inches
     x_drive.resetSensors(20);
     x_drive.setHeading(-90, true); //right 90
     x_drive.resetSensors(20);
     //moving towards next circle
     x_drive.setPos(110000); //forward 110 inches
     x_drive.resetSensors(20);
     //starting next circle
     x_drive.setHeading(90, true); //left 90
     x_drive.resetSensors(20);
     x_drive.setPos(30000); //forward 30 inches
     x_drive.resetSensors(20);
     x_drive.setHeading(90, true); //left 90
     x_drive.resetSensors(20);
     x_drive.setPos(30000); //forward 30 inches
     x_drive.resetSensors(20);
     x_drive.setHeading(90, true); //left 90
     x_drive.resetSensors(20);
     x_drive.setPos(120000); //forward 120 inches
     x_drive.resetSensors(20);
     //next circle
     x_drive.setHeading(90, true); //left 90
     x_drive.resetSensors(20);
     x_drive.setPos(70000); //forward 70 inches
     x_drive.resetSensors(20);
     x_drive.setHeading(90, true); //left 90
     x_drive.resetSensors(20);
     x_drive.setPos(30000); //forward 30 inches
     x_drive.resetSensors(20);
     x_drive.setHeading(90, true); //left 90
     x_drive.resetSensors(20);
     //return to start
     x_drive.setPos(300000); //forward 300 inches
     x_drive.resetSensors(20);
     break;

    /* --- Slalom Path --- */
    case 6:
     //start drive
     x_drive.setPos(50000); //forward 50 inches
     x_drive.resetSensors(20);
     x_drive.setHeading(45, true); //left 45
     x_drive.resetSensors(20);
     x_drive.setPos(120000); //forward 120 inches
     x_drive.resetSensors(20);
     x_drive.setHeading(-45, true); //right 45
     x_drive.resetSensors(20);
     x_drive.setPos(130000); //forward 130 inches
     x_drive.resetSensors(20);
     x_drive.setHeading(-45, true); //right 45
     x_drive.resetSensors(20);
     x_drive.setPos(90000); //forward 90 inches
     x_drive.resetSensors(20);
     //entering circle
     x_drive.setHeading(45, true); //left 45
     x_drive.resetSensors(20);
     x_drive.setPos(80000); //forward 80 inches
     x_drive.resetSensors(20);
     x_drive.setHeading(90, true); //left 90
     x_drive.resetSensors(20);
     x_drive.setPos(60000); //forward 60 inches
     x_drive.resetSensors(20);
     x_drive.setHeading(90, true); //left 90
     x_drive.resetSensors(20);
     x_drive.setPos(80000); //forward 80 inches
     x_drive.resetSensors(20);
     x_drive.setHeading(45, true); //left 45
     x_drive.resetSensors(20);
     x_drive.setPos(60000); //forward 60 inches
     x_drive.resetSensors(20);
     //returning to start
     x_drive.setHeading(-45, true); //right 45
     x_drive.resetSensors(20);
     x_drive.setPos(110000); //forward 110 inches
     x_drive.resetSensors(20);
     x_drive.setHeading(-45, true); //right 45
     x_drive.resetSensors(20);
     x_drive.setPos(70000); //forward 70 inches
     x_drive.resetSensors(20);
     x_drive.setHeading(60, true); //left 60
     x_drive.resetSensors(20);
     x_drive.setPos(60000); //forward 60 inches
     x_drive.resetSensors(20);
     break;

     /* --- Bounce Path --- */
    case 7:
     //start drive
     x_drive.setPos(40000); //forward 40 inches
     x_drive.resetSensors(20);
     x_drive.setHeading(90, true); //left 90
     x_drive.resetSensors(20);
     x_drive.setPos(70000); //forward 70 inches
     x_drive.resetSensors(20);
     //going to second point
     x_drive.setPos(-20000); //backwards 20 inches
     x_drive.resetSensors(20);
     x_drive.setHeading(-75, true); //right 75
     x_drive.resetSensors(20);
     x_drive.setPos(100000); //forward 100 inches
     x_drive.resetSensors(20);
     x_drive.setHeading(45, true);  //left 45
     x_drive.resetSensors(20);
     x_drive.setPos(45000);//forward 45 inches
     x_drive.resetSensors(20);
     x_drive.setHeading(90, true); //left 90
     x_drive.resetSensors(20);
     x_drive.setPos(120000); //forward 120 inches
     x_drive.resetSensors(20);
     //going to third point
     x_drive.setPos(-20000); //backwards 20 inches
     x_drive.resetSensors(20);
     x_drive.setHeading(-75, true); //right 75
     x_drive.resetSensors(20);
     x_drive.setPos(90); //forwards 90
     x_drive.resetSensors(20);
     x_drive.setHeading(45, true); //left 45
     x_drive.resetSensors(20);
     x_drive.setPos(80000); //forward 80 inches
     x_drive.resetSensors(20);
     x_drive.setHeading(90, true); //left 90
     x_drive.resetSensors(20);
     x_drive.setPos(120000); //forward 120 inches
     x_drive.resetSensors(20);
     x_drive.setPos(-70000); //backwards 70 inches
     x_drive.resetSensors(20);
     x_drive.setHeading(-90, true); //right 90
     x_drive.resetSensors(20);
     x_drive.setPos(60000); //forward 60 inches
     x_drive.resetSensors(20);
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
    // x_pidgey = new PigeonIMU(x_pidgeyTalon);
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
      final double STEER_K = 0.026;
      double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
      double steer_cmd = tx * STEER_K;
      double m_LimelightSteerCommand = steer_cmd;
      double area = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
      double targetVeloity = Math.abs((-1459.5* area)+10132);//10139
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
      final double STEER_K = 0.026;
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
      public void autoTrackingStartMiddle2() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(2); //LED ON
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
        final double STEER_K = 0.026;
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double steer_cmd = tx * STEER_K;
        double m_LimelightSteerCommand = steer_cmd;
        double area = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
        double targetVeloity = Math.abs((-1559.5* area)+10146);  //(-1549.5*area)+10139
        SmartDashboard.putNumber("Target Velocity", targetVeloity);
        x_turret.set(ControlMode.PercentOutput, m_LimelightSteerCommand);
        x_shootDrive.set(ControlMode.Velocity,  targetVeloity);
        if(x_shootDrive.getSelectedSensorVelocity() >  Math.abs(targetVeloity - 50)) {
          x_index.shootMode();
        }
  }

  public void autoTracking() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(2); //LED ON
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
    final double STEER_K = 0.026;
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double steer_cmd = tx * STEER_K;
    double m_LimelightSteerCommand = steer_cmd;
    double area = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    double targetVeloity = Math.abs((-1559.5* area)+10139);  //(-1549.5*area)+10139
    SmartDashboard.putNumber("Target Velocity", targetVeloity);
    x_turret.set(ControlMode.PercentOutput, m_LimelightSteerCommand);
    //x_shootDrive.set(ControlMode.Velocity,  targetVeloity);
    //if(x_shootDrive.getSelectedSensorVelocity() >  Math.abs(targetVeloity - 50)) {
    //  x_index.shootMode();
    
  } 

  public void autoDriverightTrench() {

      x_drive.driveDirect(.28, .28);

  }

  public void autoDrive() {

    x_drive.driveDirect(.29, .26);

}

  public void autoDrive2() {

    x_drive.driveDirect(.33, .30);
  }

  public void autoDriveReverseDefault(double timeout) {

    Timer timerx = new Timer();
    timerx.start();
    do {

      x_drive.driveDirect(-.30, -.30);
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
