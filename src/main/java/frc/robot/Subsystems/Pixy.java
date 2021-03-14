package frc.robot.Subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.ILoopable;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robots.RobotMap;
import frc.robot.Robots.Subsystems;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.SPILink;

public class Pixy implements ILoopable{

    static Pixy2 _pixy;
    static int blockCount;

    Joystick _joystick;

    DriveTrain _drive;

    public Pixy(){

      _joystick = RobotMap.driverJoystick;
      _drive = Subsystems.driveTrain;

    }

    public void onStart(){

        //Creates a new Pixy2 camera using SPILink
        Pixy2 _pixy = Pixy2.createInstance(new SPILink());
        //Initializes Pixy
        _pixy.init();
        //Turns the LEDS on
        _pixy.setLamp((byte) 1, (byte) 1 );
        //Sets the RGB LED to purple
        _pixy.setLED(200, 30, 255);

    }

    public void onLoop(){

        getBiggestBlock();

        if(_joystick.getRawButton(13)) {
          ArrayList<Block> blocks = _pixy.getCCC().getBlockCache();
          double xcoord = blocks.get(0).getX();
          double ycoord = blocks.get(0).getY();
          double steer = (((xcoord-39)*.01)*.5);
          double drive = -(((ycoord-51)*.01)*.5);

          _drive.arcade(drive*.02, steer*.02);
        }

    }

    public static Block getBiggestBlock() {

      _pixy = Pixy2.createInstance(new SPILink());
      _pixy.init();
      _pixy.setLamp((byte) 1, (byte) 1);

        blockCount = _pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 25);
        SmartDashboard.putNumber("Block Count", blockCount);
        System.out.println("Found" + blockCount + "blocks!");

        ArrayList<Block> blocks = _pixy.getCCC().getBlockCache();
        if (blockCount > 0) {
          double age = blocks.get(0).getAge();
          double angle = blocks.get(0).getAngle();
          double index = blocks.get(0).getIndex();
          double width = blocks.get(0).getWidth();
          double hieght = blocks.get(0).getHeight();
          double signature = blocks.get(0).getSignature();
          double xcoord = blocks.get(0).getX();
          double ycoord = blocks.get(0).getY();
          String data = blocks.get(0).toString();
    
          SmartDashboard.putNumber("Age of Target", age);
          SmartDashboard.putNumber("Angle of Target", angle);
          SmartDashboard.putNumber("Index of Target", index);
          SmartDashboard.putNumber("Width of Target", width);
          SmartDashboard.putNumber("Hieght of Target", hieght);
          SmartDashboard.putNumber("Signature of Target", signature);
          SmartDashboard.putNumber("X of Target", xcoord);
          SmartDashboard.putNumber("Y of Target", ycoord);
          SmartDashboard.putString("Data of Target", data);
        }
        
        if (blockCount <= 0) {
            return null; // If blocks were not found, stop processing
          }
      
          //ArrayList<Block> blocks = _pixy.getCCC().getBlocks(); // Gets a list of all blocks found by the Pixy2
          Block largestBlock = null;
          for (Block block : blocks) { // Loops through all blocks and finds the widest one
            if (largestBlock == null) {
              largestBlock = block;
            } else if (block.getWidth() > largestBlock.getWidth()) {
              largestBlock = block;
            }
          }
          return largestBlock;
    
    }

    public boolean isDone(){
        return false;

    }

    public void onStop() {


    } 
}