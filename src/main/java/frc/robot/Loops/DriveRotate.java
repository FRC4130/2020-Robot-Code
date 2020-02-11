package frc.robot.Loops;

import com.ctre.phoenix.ILoopable;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.robot.Robots.Subsystems;
import frc.robot.Subsystems.DriveTrain;

public class DriveRotate implements ILoopable{

    DriveTrain _drive;

    double diff;
    double target;
    double error;
    double lastErr;

    double pGain;
    double iGain;
    double dGain;

    double iZone = 10;
    double iAccum = 0;

    double acceptableErr = 3;
    int debouncedTarget = 5;
    int debounced = 0;

    double startMS = 0;
    double maxThrottle = 1;

    @Deprecated
    public DriveRotate(DriveTrain drive, double dif) {

        diff = dif;
        _drive = drive;

    }

    public DriveRotate(double dif) {

        diff = dif;
        _drive = Subsystems.driveTrain;

    }

    public DriveRotate(double dif, double acceptableEror, int debouncedtarget) {

        diff = dif;
        _drive = Subsystems.driveTrain;
        debouncedTarget = debouncedtarget;
        acceptableErr = acceptableEror;

    }

    public void onStart() {

        System.out.println("[INFO] Started Drive Rotate");

        startMS = System.currentTimeMillis();
        debounced = 0;
        target = _drive.getHeading() + diff;

        _drive.setNeutralMode(NeutralMode.Brake);

    }

    public void onLoop() {

        error = _drive.getHeading() - target;

        if (Math.abs(error) < iZone) {

            iAccum = iAccum + error;

        }
        else {

            iAccum = 0;

        }

        double turnThrottle = (pGain * error) + (iAccum * iGain) + (dGain * (error - lastErr));

        turnThrottle = turnThrottle > maxThrottle ? maxThrottle : turnThrottle < maxThrottle*-1 ? maxThrottle*-1 :turnThrottle;

        _drive.arcade(0, turnThrottle);

        lastErr = error;
    }

    public boolean isDone() {

        debounced += Math.abs(error) < acceptableErr ? 1: -1;

        debounced = debounced > debouncedTarget ? debouncedTarget : debounced < 0 ? 0 : debounced;

        if (debounced == debouncedTarget) {
			
			System.out.print("[Info] Finished Turning in ");
			System.out.print(System.currentTimeMillis()-startMS);
			System.out.print(" milliseconds with an error of ");
			System.out.print(Math.floor(error*100)/100);
			System.out.println(" degrees.");
			_drive.driveDirect(0, 0);
			
			return true;
			
		}
		
		return false;

    }

    public void onStop() {

        _drive.driveDirect(0, 0);

    }

}