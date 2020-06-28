import java.util.Arrays;
import lejos.robotics.subsumption.*;
//  import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;
//import com.cyberbotics.webots.controller.Supervisor;
import com.cyberbotics.webots.controller.Node;
import com.cyberbotics.webots.controller.Field;

public class LoadFood implements Behavior {

private MyRobot robot;
int timeStep;

private Motor motorR;  
private Motor motorL;
private boolean suppressed;
private boolean callStep;
private double foodCoords[] = new double[2];
private double foodRadius;

public LoadFood(MyRobot r, boolean cs) {
  robot = r;
  motorR = r.getMotor("right wheel motor");
  motorL = r.getMotor("left wheel motor");
  suppressed = true;
  callStep = cs;
  
  Node fs = robot.getFromDef("fs");
  foodRadius = fs.getField("foodRadius").getSFFloat();
  double[] FSpos = fs.getField("translation").getSFVec3f(); 
  foodCoords[0] = FSpos[0];
  foodCoords[1] = -FSpos[2];
         

}


public boolean takeControl() {
  if (callStep) { 
    robot.step(32);
    } 
  //Return TRUE if robot has no food and 
  // robot is inside FoodSource reach   
  if (robot.hasFood) { return false; }  
  else
  // veo a que distancia estoy del centro de la fuente de comida
  //devuelvo true si estoy dentro de su alcance
  //si no existe fuente de comida devuelvo false  
   { double[] pos = robot.currentPosition();
     return ( Math.hypot( foodCoords[0] - pos[0], foodCoords[1] - pos[1] ) < foodRadius)  ;
  }
}

public void action() {
  //double foodDir, theta , V, W;
  robot.hasFood = true;
  //el atractor ya no es la comida 
  robot.atractor = null;
  
   
  Node fs = robot.getFromDef("fs");
  Field valueField = fs.getField("value");
  int value = valueField.getSFInt32();
  System.out.println("food source value = " + value);
  if( value > 1) { value -= 1;
                   valueField.setSFInt32(value - 1); }
  else {fs.remove();}  
}    
  
public void suppress() {
  }
}