import java.util.Arrays;
import lejos.robotics.subsumption.*;
//  import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Pen;

public class Unload implements Behavior {

private MyRobot robot;
private boolean suppressed;
private boolean callStep;

/*int timeStep;

private Motor motorR;  
private Motor motorL;
private Pen pen;
private boolean callStep;
private double a;

public double[] nestCoords = {0 , 0};
//private GPS gps*/

public Unload(MyRobot r, boolean cs) {
  robot = r;
  suppressed = true;
  callStep = cs; 
  /*a = 0.3;
  pen = r.getPen("pen");
  pen.write(true);
  motorR = r.getMotor("right wheel motor");
  motorL = r.getMotor("left wheel motor");
  motorL.setPosition(Double.POSITIVE_INFINITY);
  motorR.setPosition(Double.POSITIVE_INFINITY);
  motorL.setVelocity(0);
  motorR.setVelocity(0);
  suppressed = true;
  */
}


public boolean takeControl() {
  if (callStep) { 
    robot.step(32);
    } 
  
  // calculo mi psosicion, si estoy a menos de 50 cm devuelvo true  
  double[]  myPos = robot.currentPosition();
  if ( Math.hypot(myPos[0], myPos[1]) <= 0.50 ) {
    return true;}
  else {
    return false;}
  }

public void action() {
  while (!suppressed) {
      
     
  
  }// fin del while
  // me suprimieron, hora del clean up
  
    
  }

public void suppress() {
  suppressed = true;
  }

}