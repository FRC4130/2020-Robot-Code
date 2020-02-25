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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robots.Loops;
import frc.robot.Robots.RobotMap;
import frc.robot.Robots.Subsystems;

public class Robot extends TimedRobot {

  ConcurrentScheduler teleop;

  SequentialScheduler autonTest;
  SequentialScheduler autonRight;
  SequentialScheduler autonMiddle;
  SequentialScheduler autonLeft;
  SequentialScheduler AutonFarLeft;

  int posi = 0;

  @Override
  public void robotInit() {

    RobotMap.Init();
    Subsystems.Init();

  }

  @Override
  public void autonomousInit() {

    autonTest = new SequentialScheduler(0);
    
    autonRight = new SequentialScheduler(0);
    autonMiddle = new SequentialScheduler(0);
    autonLeft = new SequentialScheduler(0);
    AutonFarLeft = new SequentialScheduler(0);

    switch (posi) {

      case 0: Loops.sTest(autonTest);

      case 1: Loops.RightAuton(autonRight);

      case 2: Loops.MiddleAuton(autonMiddle);

      case 3: Loops.LeftAuton(autonLeft);

      case 4: Loops.FarLeftAuton(AutonFarLeft);

    }

    autonTest.start();
    autonRight.start();
    autonMiddle.start();
    autonLeft.start();
    AutonFarLeft.start();

  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void disabledPeriodic() {

    if(RobotMap.driverJoystick.getRawButtonPressed(11)) {

      posi++;

    }

  }

  @Override
  public void robotPeriodic() {

    SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("Match Time", DriverStation.getInstance().getMatchTime());

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
