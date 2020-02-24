package frc.robot.Robots;

import com.ctre.phoenix.schedulers.ConcurrentScheduler;
import com.ctre.phoenix.schedulers.SequentialScheduler;

import frc.robot.Loops.ColorWheelTele;
import frc.robot.Loops.DriveDistance;
import frc.robot.Loops.DriveRotate;
import frc.robot.Loops.DriveStraight;
import frc.robot.Loops.DriveTele;
import frc.robot.Loops.IndexTele;
import frc.robot.Subsystems.Turret;

public class Loops {

    public static void sTeleop(ConcurrentScheduler teleop) {

        System.out.println("Scheduling Teleop");

        teleop.add(new DriveTele());
        teleop.add(new IndexTele());
        teleop.add(new Turret());
        // teleop.add(new ColorWheelTele());
        teleop.add(new DriveStraight());

        System.out.println("Scheduled");

    }

    public static void sTest(SequentialScheduler test) {

        test.add(new DriveRotate(90));
        test.add(new DriveDistance(300));



    }

    public static void RightAuton(SequentialScheduler Rightauton ) {



    }

    public static void MiddleAuton(SequentialScheduler Middleauton) {


    }

    public static void LeftAuton(SequentialScheduler Leftauton) {


        
    }

    public static void FarLeftAuton(SequentialScheduler FarLeftauton) {

        
    }


}