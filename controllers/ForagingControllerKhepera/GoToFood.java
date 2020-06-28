import java.util.Arrays;
import lejos.robotics.subsumption.*;
//  import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;
//import com.cyberbotics.webots.controller.Supervisor;
import com.cyberbotics.webots.controller.Node;
import com.cyberbotics.webots.controller.Field;

public class GoToFood implements Behavior {

private MyRobot robot;
int timeStep;

private Motor motorR;  
private Motor motorL;
private boolean suppressed;
private boolean callStep;
private double foodCoords[] = new double[2];
private double foodReach;

public GoToFood(MyRobot r, boolean cs) {
  robot = r;
  motorR = r.getMotor("right wheel motor");
  motorL = r.getMotor("left wheel motor");
  suppressed = true;
  callStep = cs;
  
  Node fs = robot.getFromDef("fs");
  foodReach = fs.getField("reachRadius").getSFFloat();
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
         return ( Math.hypot( foodCoords[0] - pos[0], foodCoords[1] - pos[1] ) < foodReach)  ;
  }
}

public void action() {
  double foodDir, theta , V, W, wl, wr;
  suppressed = false;
  robot.atractor = foodCoords.clone();
  robot.atractorIsObject = true; 
  while (!suppressed) {  
     //Food vector (robot based)
     double[] myPos = robot.currentPosition();
     
     foodDir = Math.atan2(foodCoords[1] - myPos[1], foodCoords[0] - myPos[0]);
     theta = robot.turningAngle(robot.currentDirection(), foodDir);
     
     W = Math.signum(theta)*theta*theta*(robot.Wmax / (Math.PI*Math.PI));
     V = robot.computeAbsV(W);
  
     wl = robot.computewl(V,W);
     wr = robot.computewr(V,W);
    motorL.setVelocity(wl);
    motorR.setVelocity(wr);
  
  }// fin del while
   

}

public void suppress() {
  suppressed = true;
  
  }

}