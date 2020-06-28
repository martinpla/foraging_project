import java.util.Arrays;
import lejos.robotics.subsumption.*;
//  import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;
//import com.cyberbotics.webots.controller.Supervisor;
import com.cyberbotics.webots.controller.Node;
import com.cyberbotics.webots.controller.Field;

public class UnloadFood implements Behavior {

private MyRobot robot;
int timeStep;

private Motor motorR;  
private Motor motorL;
private boolean suppressed;
private boolean callStep;
private double nestRadius;
private double[] nestCoords;

public UnloadFood(MyRobot r, boolean cs) {
  robot = r;
  motorR = r.getMotor("right wheel motor");
  motorL = r.getMotor("left wheel motor");
  suppressed = true;
  callStep = cs;
  
  Node nest = robot.getFromDef("nest");
  nestRadius = nest.getField("radius").getSFFloat();
  double[] nestPos = nest.getField("translation").getSFVec3f(); 
  nestCoords = new double[2];
  nestCoords[0] = nestPos[0];
  nestCoords[1] = -nestPos[2];
}

public boolean takeControl() {
  if (callStep) { 
    robot.step(32);
    } 
  //Return TRUE if robot has food and 
  // robot is inside nest   
  if (!robot.hasFood) { return false; }  
  else
  //devuelvo true si estoy dentro del nido
   { double[] pos = robot.currentPosition();
     return ( Math.hypot( nestCoords[0] - pos[0], nestCoords[1] - pos[1] ) < nestRadius)  ;
  }
}

public void action() {
  //double foodDir, theta , V, W;
  Node nest = robot.getFromDef("nest");
  Field valueField = nest.getField("value");
  int value = valueField.getSFInt32();
  value += 1;
  System.out.println("Ahora el nido tiene value = " + value);
  valueField.setSFInt32(value); 
  robot.hasFood = false;
  //atractor ya no es el nido
  robot.atractor = null ; 
}    
  
public void suppress() {
  System.out.println("Unload supress");
  }
}