import java.util.Arrays;
import lejos.robotics.subsumption.*;
//  import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Pen;

public class GoToNest implements Behavior {

private MyRobot robot;
int timeStep;

private Motor motorR;  
private Motor motorL;
private Pen pen;
private boolean suppressed;
private boolean callStep;


//private GPS gps
public GoToNest(MyRobot r, boolean cs) {
  robot = r;
  motorR = r.getMotor("right wheel motor");
  motorL = r.getMotor("left wheel motor");
  
  suppressed = true;
  callStep = cs;
}


public boolean takeControl() {
  if (callStep) { 
    robot.step(32);
    } 
  
  return robot.hasFood;
  }

public void action() {
  //System.out.println("Go to nest Action");
  double[] myPos;
  double nestDir, theta , V, W, wr, wl;
  suppressed = false;
  robot.atractor = robot.nestCoords.clone();
  robot.atractorIsObject = true;
  while (!suppressed) {
    
     myPos = robot.currentPosition();
     nestDir = Math.atan2(robot.nestCoords[1]- myPos[1], robot.nestCoords[0]- myPos[0] );
     double currentDir = robot.currentDirection();
     theta = robot.turningAngle(currentDir, nestDir);
     
     
     W = Math.signum(theta)*theta*theta*(robot.Wmax / (Math.PI*Math.PI));
     V = robot.computeAbsV(W);
     
     wl = robot.computewl(V,W);
     wr = robot.computewr(V,W);
    motorL.setVelocity(wl);
    motorR.setVelocity(wr);
  
     
  
  }// fin del while
  // me suprimieron, freno el robot ?
  //motorL.setVelocity(0);
  //motorR.setVelocity(0);
    
  }

public void suppress() {
  suppressed = true;
  
  }

}