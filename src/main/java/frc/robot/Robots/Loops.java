package frc.robot.Robots;

import com.ctre.phoenix.schedulers.ConcurrentScheduler;
import com.ctre.phoenix.schedulers.SequentialScheduler;

import frc.robot.Loops.ClimbTele;
import frc.robot.Loops.DriveDistance;
import frc.robot.Loops.DriveRotate;
import frc.robot.Loops.DriveTele;
import frc.robot.Loops.IndexTele;
import frc.robot.Subsystems.ColorWheel;
import frc.robot.Subsystems.Pixy;
import frc.robot.Subsystems.Turret;

public class Loops {

    public static void sTeleop(ConcurrentScheduler teleop) {

        System.out.println("Scheduling Teleop");

        teleop.add(new DriveTele());
        teleop.add(new IndexTele());
        teleop.add(new Turret());
        teleop.add(new ColorWheel());
        //teleop.add(new DriveStraight());
        teleop.add(new ClimbTele());
        //teleop.add(new Pixy());

        System.out.println("Scheduled");

    }

    public static void sTest(SequentialScheduler test) {

        test.add(new DriveRotate(90));
        test.add(new DriveDistance(300));


    }

    public static void FarRightAuton(SequentialScheduler FarRightAuton) {



    }
    
    public static void RightAuton(SequentialScheduler Rightauton ) {



    }

    public static void MiddleAuton(SequentialScheduler Middleauton) {


    }

    public static void LeftAuton(SequentialScheduler Leftauton) {
        
    }

    public static void DefaultForwardAuton(SequentialScheduler ForwardAuton) {

        ForwardAuton.add(new DriveDistance(60000));


    }

    public static void DefaultBackwardsAuton(SequentialScheduler BackwardsAuton) {

    }

}