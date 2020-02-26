/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.schedulers.ConcurrentScheduler;
import com.ctre.phoenix.schedulers.SequentialScheduler;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robots.Loops;
import frc.robot.Robots.RobotMap;
import frc.robot.Robots.Subsystems;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.Index;
import frc.robot.Subsystems.Turret;

public class Robot extends TimedRobot {

  String[] pos = {"Test", "Right", "Middle", "Left", "Far Right", "Forward", "Reverse"};

  ConcurrentScheduler teleop;

  DriveTrain _drive;
  Turret _turret;
  Index _index;

  // SequentialScheduler AutonTest;
  // SequentialScheduler AutonRight;
  // SequentialScheduler AutonMiddle;
  // SequentialScheduler AutonLeft;
  // SequentialScheduler AutonFarRight;
  // SequentialScheduler AutonForward;
  // SequentialScheduler AutonBackwards;

  String gameData;

  int posi = 0;

  @Override
  public void robotInit() {

    RobotMap.Init();
    Subsystems.Init();

    _drive = Subsystems.driveTrain;
    _turret = Subsystems.turret;
    _index = Subsystems.index;

  }

  @Override
  public void autonomousInit() {

    // AutonTest = new SequentialScheduler(0);
    // AutonRight = new SequentialScheduler(0);
    // AutonMiddle = new SequentialScheduler(0);
    // AutonLeft = new SequentialScheduler(0);
    // AutonFarRight = new SequentialScheduler(0);
    // AutonForward = new SequentialScheduler(0);
    // AutonBackwards = new SequentialScheduler(0);

    // switch (posi) {

    //   case 0: Loops.sTest(AutonTest);

    //   case 1: Loops.FarRightAuton(AutonFarRight);

    //   case 2: Loops.RightAuton(AutonRight);

    //   case 3: Loops.MiddleAuton(AutonMiddle);

    //   case 4: Loops.LeftAuton(AutonLeft);

    //   case 5: Loops.DefaultForwardAuton(AutonForward);

    //   case 6: Loops.DefaultBackwardsAuton(AutonBackwards);

    //}

  //   AutonTest.start();
  //   AutonFarRight.start();
  //   AutonRight.start();
  //   AutonMiddle.start();
  //   AutonLeft.start();
  //   AutonForward.start();
  //   AutonBackwards.start();

   }

  @Override
  public void autonomousPeriodic() {
      
      _turret.AutonLimelight();
      Timer.delay(5);
     _drive.driveDirect(.15, -.15);
     Timer.delay(2);
     _drive.driveDirect(0, 0);
     _index.Intake(.60);
     Timer.delay(15);
  
  }

  @Override
  public void disabledPeriodic() {

    if(RobotMap.driverJoystick.getRawButtonPressed(9)) {

      posi++;

    }

  }

  @Override
  public void robotPeriodic() {

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

}
