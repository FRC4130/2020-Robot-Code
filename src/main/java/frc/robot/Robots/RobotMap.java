package frc.robot.Robots;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;

public class RobotMap {


    /* --- Motor Objects --- */
    public static TalonFX leftDrive;
    public static TalonFX leftDrive2;

    public static TalonFX rightDrive;
    public static TalonFX rightDrive2;

    public static TalonFX Shooter1;
    public static TalonFX Shooter2;

    public static PigeonIMU pigeon;

    public static TalonSRX turret;

    public static TalonSRX Index1;
    public static TalonSRX Index2;
    public static TalonSRX Index3;
    public static TalonSRX Index4;
    public static TalonSRX Index5;

    /* --- Solenoid Objects --- */
    public static DoubleSolenoid intakeLift;

    /* --- Joystick Objects --- */
    public static Joystick driverJoystick;
    public static Joystick operatorJoystick;

    /* --- Motor Controller CAN ID's --- */
    public static final int kLeftDriveID = 1;
    public static final int kLeftDrive2ID = 2;
    public static final int kRightDriveID = 3;
    public static final int kRightDrive2ID = 4;

    public static final int kIndex1ID = 5;
    public static final int kIndex2ID = 6;
    public static final int kIndex3ID = 7;
    public static final int kIndex4ID = 8;
    public static final int kIndex5ID = 9;
    public static final int kShooter1ID = 10;
    public static final int kShooter2ID = 11;
    public static final int kTurretID = 12;

    /* --- Solenoid Objects --- */
    public static final int kIntakeDownID = 0;
    public static final int kIntakeUpID = 1;

    /* --- Joystick ID's --- */
    public static final int kDriverJoysickID = 0;
    public static final int kOperatorJoystickID = 1;



    public static void Init() {

        //Drivetrain
        leftDrive = new TalonFX(kLeftDriveID);
        leftDrive2 = new TalonFX(kLeftDrive2ID);

        rightDrive = new TalonFX(kRightDriveID);
        rightDrive2 = new TalonFX(kRightDrive2ID);

        //Pigeon
        pigeon = new PigeonIMU(1);

        //Shooter
        Shooter1 = new TalonFX(kShooter1ID);
        Shooter2 = new TalonFX(kShooter2ID);

        //Turret
        turret = new TalonSRX(kTurretID);

        //Index
        Index1 = new TalonSRX(kIndex1ID);
        Index2 = new TalonSRX(kIndex2ID);
        Index3 = new TalonSRX(kIndex3ID);
        Index4 = new TalonSRX(kIndex4ID);
        Index5 = new TalonSRX(kIndex5ID);

        //Joysticks
        driverJoystick = new Joystick(kDriverJoysickID);
        operatorJoystick = new Joystick(kOperatorJoystickID);

        //Solenoid for Intake
        intakeLift = new DoubleSolenoid(kIntakeDownID, kIntakeUpID);


    }
}