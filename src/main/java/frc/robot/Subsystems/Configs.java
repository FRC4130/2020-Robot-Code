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
        leftDrive.slot0.kF = 0;
        leftDrive.slot0.kP = 0;
        leftDrive.slot0.kD = 0;
        leftDrive.motionAcceleration = 200;
        leftDrive.motionCruiseVelocity = 200;

        /* --- Right Drive --- */
        rightDrive.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        rightDrive.slot0.kF = 0;
        rightDrive.slot0.kP = 0;
        rightDrive.slot0.kD = 0;
        rightDrive.motionAcceleration = 200;
        rightDrive.motionCruiseVelocity = 200;

        /* --- Shooter --- */
        shooter.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        shooter.slot0.kF = 0;
        shooter.slot0.kP = 0;
        shooter.slot0.kD = 0;
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