package frc.robot.Subsystems;

import com.ctre.phoenix.ILoopable;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robots.RobotMap;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;


public class ColorWheel implements ILoopable{

    
  TalonSRX _colorDrive =  RobotMap.ColorWheel;
  //private final int kTimeoutMS = 10;

  Joystick _joystick = new Joystick(0); 
  double kTargetAngleDegrees = 0.0;

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);


  public ColorWheel() {
    _colorDrive.setInverted(false);
    _colorDrive.setNeutralMode(NeutralMode.Brake);
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);    
  }

  public void onStart() {
  }

  public void onLoop() {
    colorWheelStop();
    colorWheelSpin();
  }



  public void colorWheelStop(){
    Color detectedColor = m_colorSensor.getColor();
    double IR = m_colorSensor.getIR();
    int proximity = m_colorSensor.getProximity();
    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } 
    else if (match.color == kRedTarget) {
      colorString = "Red";
    } 
    else if (match.color == kGreenTarget) {
      colorString = "Green";
    } 
    else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } 
    else {
      colorString = "Unknown";
    }
    SmartDashboard.putString("Detected Color", colorString);
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putNumber("IR", IR);
    SmartDashboard.putNumber("Proximity", proximity);
    SmartDashboard.putNumber("Left Velocity", _colorDrive.getSelectedSensorVelocity(0));
    SmartDashboard.putNumber("Left Position", _colorDrive.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("POV Value", _joystick.getPOV());
    detectedColor = m_colorSensor.getColor();
    match = m_colorMatcher.matchClosestColor(detectedColor);

    if (_joystick.getPOV() > -1 && _joystick.getPOV() <1) {
      if (match.color == kBlueTarget) {
        double driveThrottle = 0;
        _colorDrive.set(ControlMode.PercentOutput, driveThrottle);
      }
      else {
        double driveThrottle = .25;
        _colorDrive.set(ControlMode.PercentOutput, driveThrottle);
      }
    }
    else if (_joystick.getPOV() >89 && _joystick.getPOV() <91) {
      if (match.color == kGreenTarget) {
        double driveThrottle = 0;
        _colorDrive.set(ControlMode.PercentOutput, driveThrottle);
      }
      else{
        double driveThrottle = .25;
        _colorDrive.set(ControlMode.PercentOutput, driveThrottle);
      }
    }
    else if (_joystick.getPOV() >179 && _joystick.getPOV() <181) {
      if (match.color == kRedTarget) {
        double driveThrottle = 0;
        _colorDrive.set(ControlMode.PercentOutput, driveThrottle);
      }
      else{
        double driveThrottle = .25;
        _colorDrive.set(ControlMode.PercentOutput, driveThrottle);
      }
    }
    else if (_joystick.getPOV() >269 && _joystick.getPOV() <271) {
      if (match.color == kYellowTarget) {
        double driveThrottle = 0;
        _colorDrive.set(ControlMode.PercentOutput, driveThrottle);
      }
      else{
        double driveThrottle = .25;
        _colorDrive.set(ControlMode.PercentOutput, driveThrottle);
      }
    }
  }
  


  public void colorWheelSpin(){        
    int Flag1 = 0;
    if (_joystick.getRawButton(2)) {
      Color detectedColor = m_colorSensor.getColor();
      ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
      if (match.color == kBlueTarget) {
        Flag1 = 1;
      }
      else if (match.color == kRedTarget) {
        Flag1 = 2;
      } 
      else if (match.color == kGreenTarget) {
        Flag1 = 3;
      } 
      else if (match.color == kYellowTarget) {
        Flag1 = 4;
      }
    }

    if (Flag1 == 1){      
      Color detectedColor = m_colorSensor.getColor();
      ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
      String colorString2;
      for (int i=8; i>1; i--){
        do {
          double driveThrottle = .20;
          _colorDrive.set(ControlMode.PercentOutput, driveThrottle);
          detectedColor = m_colorSensor.getColor();
          match = m_colorMatcher.matchClosestColor(detectedColor);
          colorString2 = "Blue";      
          SmartDashboard.putString("Detected Color", colorString2);
          SmartDashboard.putNumber("Loop Count", i);
        }
        while (match.color == kBlueTarget);
        do {
          double driveThrottle = .20;
          _colorDrive.set(ControlMode.PercentOutput, driveThrottle);
          detectedColor = m_colorSensor.getColor();
          match = m_colorMatcher.matchClosestColor(detectedColor);
          colorString2 = "Blue";      
          SmartDashboard.putString("Detected Color", colorString2);
          SmartDashboard.putNumber("Loop Count", i);
          }
        while (!(match.color == kBlueTarget));
      }  
    }

    if (Flag1 == 2){      
      Color detectedColor = m_colorSensor.getColor();
      ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
      String colorString2;
      for (int i=8; i>1; i--){
        do {
          double driveThrottle = .20;
          _colorDrive.set(ControlMode.PercentOutput, driveThrottle);
          detectedColor = m_colorSensor.getColor();
          match = m_colorMatcher.matchClosestColor(detectedColor);
          colorString2 = "Red";      
          SmartDashboard.putString("Detected Color", colorString2);
          SmartDashboard.putNumber("Loop Count", i);
        }
        while (match.color == kRedTarget);
        do {
          double driveThrottle = .20;
          _colorDrive.set(ControlMode.PercentOutput, driveThrottle);
          detectedColor = m_colorSensor.getColor();
          match = m_colorMatcher.matchClosestColor(detectedColor);
          colorString2 = "Red";      
          SmartDashboard.putString("Detected Color", colorString2);
          SmartDashboard.putNumber("Loop Count", i);
          }
        while (!(match.color == kRedTarget));
      }  
    }

    if (Flag1 == 3){  
      Color detectedColor = m_colorSensor.getColor();
      ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
      String colorString2;
      for (int i=8; i>1; i--){
        do {
          double driveThrottle = .20;
          _colorDrive.set(ControlMode.PercentOutput, driveThrottle);
          detectedColor = m_colorSensor.getColor();
          match = m_colorMatcher.matchClosestColor(detectedColor);
          colorString2 = "Green";      
          SmartDashboard.putString("Detected Color", colorString2);
          SmartDashboard.putNumber("Loop Count", i);
        }
        while (match.color == kGreenTarget);
        do {
          double driveThrottle = .20;
          _colorDrive.set(ControlMode.PercentOutput, driveThrottle);
          detectedColor = m_colorSensor.getColor();
          match = m_colorMatcher.matchClosestColor(detectedColor);
          colorString2 = "Green";      
          SmartDashboard.putString("Detected Color", colorString2);
          SmartDashboard.putNumber("Loop Count", i);
          }
        while (!(match.color == kGreenTarget));
      }  
    }

    if (Flag1 == 4){ 
      Color detectedColor = m_colorSensor.getColor();
      ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
      String colorString2;
      for (int i=8; i>1; i--){
        do {
          double driveThrottle = .20;
          _colorDrive.set(ControlMode.PercentOutput, driveThrottle);
          detectedColor = m_colorSensor.getColor();
          match = m_colorMatcher.matchClosestColor(detectedColor);
          colorString2 = "Yellow";      
          SmartDashboard.putString("Detected Color", colorString2);
          SmartDashboard.putNumber("Loop Count", i);
        }
        while (match.color == kYellowTarget);
        do {
          double driveThrottle = .20;
          _colorDrive.set(ControlMode.PercentOutput, driveThrottle);
          detectedColor = m_colorSensor.getColor();
          match = m_colorMatcher.matchClosestColor(detectedColor);
          colorString2 = "Yellow";      
          SmartDashboard.putString("Detected Color", colorString2);
          SmartDashboard.putNumber("Loop Count", i);
          }
        while (!(match.color == kYellowTarget));
      }  
    }
  }


  public boolean isDone() {
    return false;
  }

  public void onStop() {
    _colorDrive.set(ControlMode.PercentOutput, 0);
  }
}