package frc.robot.Robots;

import com.ctre.phoenix.schedulers.ConcurrentScheduler;
import com.ctre.phoenix.schedulers.SequentialScheduler;

import frc.robot.Loops.DriveDistance;
import frc.robot.Loops.DriveRotate;
import frc.robot.Loops.DriveTele;
import frc.robot.Loops.IndexTele;

public class Loops {

    public static void sTeleop(ConcurrentScheduler teleop) {

        System.out.println("Scheduling Teleop");

        teleop.add(new DriveTele());
        teleop.add(new IndexTele());

        System.out.println("Scheduled");

    }

    public static void sTest(SequentialScheduler auton) {

        auton.add(new DriveRotate(90));
        auton.add(new DriveDistance(300));



    }


}