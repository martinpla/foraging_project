import java.util.Arrays;
import lejos.robotics.subsumption.*;
//  import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Pen;
import com.cyberbotics.webots.controller.DistanceSensor;

public class Avoid implements Behavior {

private MyRobot robot;
private boolean suppressed;
private boolean callStep;
private Motor motorR;  
private Motor motorL;
double W,V,wr,wl, theta, atractorDir ;
double[] sensorValues, force, myPos;
static double[] radius = {900, 900, 900, 900, 900};
static double[] repulsionDir = { -0.5*Math.PI, -0.75*Math.PI, Math.PI , 0.75*Math.PI, 0.5*Math.PI };
  

public Avoid(MyRobot r, boolean cs) {
  robot = r;
  suppressed = true;
  callStep = cs; 
  motorR = r.getMotor("right wheel motor");
  motorL = r.getMotor("left wheel motor");
  motorL.setPosition(Double.POSITIVE_INFINITY);
  motorR.setPosition(Double.POSITIVE_INFINITY);
  motorL.setVelocity(0);
  motorR.setVelocity(0);
  suppressed = true;
  force = new double[2];
}


public boolean takeControl() {
  if (callStep) { 
    robot.step(32);
    } 
  
  System.out.println("Avoid  takeControl  "); // + sensorValue[2]);
  if(!suppressed) {return true; }
  sensorValues = robot.getSensorValues();
  
  
  System.out.println("tk sensorValues = " + sensorValues[0] + ", " + sensorValues[1]+ ", " + sensorValues[2] + ", " + sensorValues[3]+ ", " + sensorValues[4]);
  for (int i = 0; i < sensorValues.length; i++) {  
    if (sensorValues[i] > radius[i]) {
      System.out.println("avoid tk = true");
  
      return true;
    }
  }
  return false;
}  

public void action() {
  double[] force = new double[2];
  //double theta, atractorDir;
  //suppressed = false;
 // while (!suppressed) {
  
  sensorValues = robot.getSensorValues();
  
  // Inicializo la fuerza resultante en el atractor
  // el atractor depende de cual era el ultimo comportamiento de transporte
  if(robot.atractor != null) {
    if (robot.atractorIsObject) {
       myPos = robot.currentPosition();
       atractorDir = Math.atan2(robot.atractor[1]- myPos[1], robot.atractor[0]- myPos[0] );
       theta = robot.turningAngle(robot.currentDirection() , atractorDir);
       force[0] += Math.cos(theta); 
       force[1] += Math.sin(theta); 
       }
    else 
      // Si el atractor no era un objecto sino que era direccinal
      // significa que era deambular. Entonces podemos tomar currentDir mejor que theta   
      // pero es mejor pedirla ahora que el valor a que pueda haber dejado Deambular 
      { force[0] += Math.cos( robot.currentDirection()/2);
        force[1] += Math.sin( robot.currentDirection()/2); 
      }
  }  
 
  //Para cada sensor, si su valor está dentro de la burbuja
  //calculo su vector repulsor y lo sumo a fuerza
  for (int i = 0; i < sensorValues.length; i++) {  
    if (sensorValues[i] > radius[i]) {
    // el vector repulsor tiene direccion repulsionDir[i]
    //force[0] += Math.cos(repulsionDir[i])* sensorValues[i]/1023;
    //force[1] += Math.sin(repulsionDir[i])* sensorValues[i]/1023;
    force[0] += Math.cos(repulsionDir[i]);
    force[1] += Math.sin(repulsionDir[i]);
    
    } 
  }
  
  //ajusto W y V segun la fuerza resultante 
  theta = Math.atan2( force[1], force[0] );
  System.out.println("dir esquive(en grados) theta = " + Math.toDegrees(theta));
   
  //W = Math.signum(theta)* 3.9 ;
  W = Math.signum(theta)*theta*theta*(robot.Wmax / (Math.PI*Math.PI));
  V = robot.computeAbsV(W);
  //W = 1;   
  wr = robot.computewr( V ,W);  
  wl = robot.computewl( V , W );
  motorL.setVelocity(wl);
  motorR.setVelocity(wr);
   
  //}// fin del while
  
  }

public void suppress() {
  System.out.println("Avoid suppress");
  suppressed = true;
  }

}