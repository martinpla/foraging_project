import java.util.Arrays;
import lejos.robotics.subsumption.*;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;

public class GoToNest implements Behavior {

private MyRobot robot;
int timeStep;

private Motor motorR, motorL;  
private boolean suppressed;
private boolean callStep;
private double cellSize;

public GoToNest(MyRobot r, boolean cs) {
  robot = r;
  motorR = r.getMotor("right wheel motor");
  motorL = r.getMotor("left wheel motor");
  
  suppressed = true;
  callStep = cs;
  cellSize = r.cellSize;
}


public boolean takeControl() {
  if (callStep) { 
    robot.step(32);
    } 
  
  return robot.hasFood;
  }

public void action() {
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
    
    //release pheromone if current cell distinct from last marked cell
    int[] currentCell = { (int) Math.round( myPos[0]/cellSize), (int) Math.round( myPos[1]/cellSize)  };
    if( ! Arrays.equals(currentCell, robot.lastMarkedCell)) {
                //si currentCell es distinto de mi celda actual
                //significa que entré a una nueva celda. Actualizo currentCell
                robot.lastMarkedCell = currentCell.clone();
                robot.releasePheromone(2);
                // Si vengo con comida dejo dos feromonas en la nueva celda
                //if (hasFood)  { releasePheromone(2); } 
            }
     
  
  }// fin del while
}

public void suppress() {
  suppressed = true;
  }

}