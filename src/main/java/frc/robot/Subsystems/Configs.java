package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;

public class Configs {

    public static TalonFXConfiguration leftDrive = new TalonFXConfiguration();
    public static TalonFXConfiguration rightDrive = new TalonFXConfiguration();
    public static TalonFXConfiguration shooter = new TalonFXConfiguration();
    public static TalonSRXConfiguration turret = new TalonSRXConfiguration();
    public static VictorSPXConfiguration intake = new VictorSPXConfiguration();

    private static Configs _instance = new Configs();

    private Configs() {

        /* --- Left Drive --- */
        leftDrive.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        leftDrive.slot0.kF = 0.0529;
        leftDrive.slot0.kP = 0.00289;
        leftDrive.motionAcceleration = 20000;
        leftDrive.motionCruiseVelocity = 40000;

        /* --- Right Drive --- */
        rightDrive.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        rightDrive.slot0.kF = 0.0529;
        rightDrive.slot0.kP = 0.00289;
        rightDrive.motionAcceleration = 20000;
        rightDrive.motionCruiseVelocity = 40000;

        /* --- Shooter --- */
        shooter.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
        shooter.slot0.kF = 0.052;
        shooter.slot0.kP = 0.59;
        shooter.slot0.kD = 12;
        shooter.motionAcceleration = 200;
        shooter.motionCruiseVelocity = 200;

        /* --- Turret --- */
        turret.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
        turret.slot0.kF = 0;
        turret.slot0.kP = 0;
        turret.slot0.kD = 0;
        turret.motionAcceleration = 200;
        turret.motionCruiseVelocity = 200;

    }

    public Configs get() {

        return _instance;

    }

}