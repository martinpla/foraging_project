import java.util.Arrays;
import java.util.Random;
import lejos.robotics.subsumption.*;
//  import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Pen;
import com.cyberbotics.webots.controller.DistanceSensor;
import java.util.Date;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;


public class Wander implements Behavior {

private MyRobot robot;
private boolean suppressed;
private boolean callStep;
private boolean working;
private Motor motorR;  
private Motor motorL;
private int counter =0;
private double lapse, wanderDir;  
private double[] turnDirs = {-120, -90, -60 - 30, 0, 30, 60, 90, 120}; 
public double W,V,wr,wl, theta, nestDir;
private Random rand;

static ScheduledExecutorService ses;  


public Wander(MyRobot r, boolean cs, long l) {
  robot = r;
  suppressed = true;
  callStep = cs; 
  suppressed = true;
 
  lapse = l;
  V = 0.05;
  W = 0;
  theta = 0;
  motorR = r.getMotor("right wheel motor");
  motorL = r.getMotor("left wheel motor");

  wanderDir = r.currentDirection();
  ses = Executors.newSingleThreadScheduledExecutor();
  ses.scheduleWithFixedDelay(new Runnable() {
            @Override
            public void run() {
                //si el comportamiento esta activo aumento el contador
                if(!suppressed) {
                  counter +=1;
                  if(counter == lapse) {
                    counter = 0;
                    //sorteo nueva wanderDir 
                    int i = Math.round((float) Math.random()*(turnDirs.length - 1));
                    wanderDir = robot.currentDirection() + Math.toRadians(turnDirs[i]);
                    System.out.println("new wanderDir " + Math.toDegrees(wanderDir));
                    robot.atractor[0] = Math.cos(wanderDir);
                    robot.atractor[1] = Math.sin(wanderDir);
                  }  
                }

                //int i = rand.nextInt(turnDirs.length + 1);
                
            }
        }, l, l, TimeUnit.SECONDS); 

}


public boolean takeControl() {
  //System.out.println("wander takeControl");
  if (callStep) { 
    robot.step(32);
    } 
  if(!robot.hasFood) {return true;}
  else { return false;}
  
  }
  

public void action() {
  suppressed = false;
  
  System.out.println("B 1");
  robot.atractorIsObject = false;
  System.out.println("B 2");
  if(robot.atractor == null) { robot.atractor = new double[2];}
  robot.atractor[0] = Math.cos(wanderDir);
  System.out.println("B 3");

  robot.atractor[1] = Math.sin(wanderDir);
  System.out.println("B 4");

 //System.out.println("Wander ACtion");
  while (!suppressed) {
    double currDir = robot.currentDirection();
    theta = robot.turningAngle(robot.currentDirection(), wanderDir);     
      System.out.println("B 5");

    W = Math.signum(theta)*theta*theta*(robot.Wmax / (Math.PI*Math.PI));
    V = robot.computeAbsV(W);
     
    wl = robot.computewl(V,W);
    wr = robot.computewr(V,W);
    motorL.setVelocity(wl);
    motorR.setVelocity(wr);
  
   
    }// fin del while
    
    //robot.atractor = null; 

}

public void suppress() {
  System.out.println("Wander suppress");
  suppressed = true;
  }

}