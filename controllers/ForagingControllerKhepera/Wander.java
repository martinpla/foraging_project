import java.util.Arrays;
import java.util.Random;
import lejos.robotics.subsumption.*;
//  import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;
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
private Motor motorR, motorL ;


private double Dact, Dant;  //Orientacion actual y orientacion anterior
private boolean DantIsValid; // indica si orientacion anterior es valida
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
                    theta = Math.toRadians(turnDirs[i]);
                    //wanderDir = robot.currentDirection() + Math.toRadians(turnDirs[i]);
                    //System.out.println("new theta =  " + Math.toDegrees(theta));
                    //robot.atractor[0] = Math.cos(wanderDir);
                    //robot.atractor[1] = Math.sin(wanderDir);
                    DantIsValid = false;
                  }  
                }

                //int i = rand.nextInt(turnDirs.length + 1);
                
            }
        }, l, l, TimeUnit.SECONDS); 

}


public boolean takeControl() {
 if (callStep) { 
    robot.step(32);
    } 
 return true;
 }
  

public void action() {
  suppressed = false;
  robot.atractorIsObject = false;
  
  if(robot.atractor == null) { 
    robot.atractor = new double[2];}
  while (!suppressed) {
      Dact = robot.currentDirection();
      if(Math.abs(theta) > Math.PI/2) {
        theta = 0;
        DantIsValid = false; 
        }
      else {
        if( DantIsValid )
           { theta -= Dact - Dant; } //update theta : theta = what is left to turn now 
        else 
            { DantIsValid = true ; }
        Dant = Dact;   
      }  
      //aplies W depending of theta 
      W = Math.signum(theta)*theta*theta*(robot.Wmax / (Math.PI*Math.PI));
      V = robot.computeAbsV(W);
      wl = robot.computewl(V,W);
      wr = robot.computewr(V,W);
      motorL.setVelocity(wl);
      motorR.setVelocity(wr);
      
      
    }// fin del while
    
    //wander is going to loose control
    
    DantIsValid = false; 
    // dejo theta en atractor
    robot.atractor[0] = Math.cos(theta); 
    robot.atractor[1] = Math.sin(theta);
    robot.atractorIsObject = false; 

}

public void suppress() {
  suppressed = true;
  }

}