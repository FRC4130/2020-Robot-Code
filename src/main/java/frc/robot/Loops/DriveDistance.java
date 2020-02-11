package frc.robot.Loops;

import com.ctre.phoenix.ILoopable;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robots.Subsystems;
import frc.robot.Subsystems.DriveTrain;

public class DriveDistance implements ILoopable{

    private double distanceNative;
    private double distanceInches;
    private DriveTrain _drive;

    private double acceptableError = 4000;

    public DriveDistance(DriveTrain driveTrain, double inches) {

        distanceInches = inches;
        _drive = Subsystems.driveTrain;

    }

    public DriveDistance(double inches) {

        distanceInches = inches;
        _drive = Subsystems.driveTrain;

    }

    public void onStart() {

        _drive.resetSensors(20);

        System.out.print("[INFO] Driving");
        System.out.println(distanceInches);

        _drive.setNeutralMode(NeutralMode.Brake);

        distanceNative = _drive.distanceToRotations(distanceInches);

    }

    public void onLoop() {

        _drive.setPosLeft(distanceNative);
        _drive.setPosRight(distanceNative);

    }

    public boolean isDone() {

        boolean leftAtPos = Math.abs(distanceNative - _drive.getLeftPos()) <= acceptableError;
        boolean rightAtPos = Math.abs(distanceNative - _drive.getRightPos()) <= acceptableError;

        SmartDashboard.putNumber("Left Error", distanceNative - _drive.getLeftPos());
        SmartDashboard.putNumber("Right Error", distanceNative - _drive.getRightPos());

        if(leftAtPos && rightAtPos) {
            System.out.println("[INFO] Dinished Driving for Distance");
            return true;

        }

        return false;

    }

    public void onStop() {

        System.out.println("[WARNING] Driving for distance was stopped");

    }

}