import java.util.Arrays;
import lejos.robotics.subsumption.*;
//  import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Pen;
import com.cyberbotics.webots.controller.DistanceSensor;

public class Avoid2 implements Behavior {

private MyRobot robot;
private boolean suppressed;
private boolean callStep;
private boolean working;
private Motor motorR;  
private Motor motorL;
double W,V,wr,wl, theta, nestDir;
double[] sensorValues;
static double[] radius = {1000, 1000, 1000, 1000};
//Los sensores IR apuntan a 42.85º, 13.3º, -13.3º y -42.85º respecto al frente del robot
//Las  direcciones de repulsion son diametralmente opuestas  
static double[] repulsionDir = { -Math.toRadians(137.15), -Math.toRadians(166.7), Math.toRadians(166.7), Math.toRadians(137.15) };
  

public Avoid2(MyRobot r, boolean cs) {
  robot = r;
  suppressed = true;
  callStep = cs; 
  working = false;
  motorR = r.getMotor("right wheel motor");
  motorL = r.getMotor("left wheel motor");
  motorL.setPosition(Double.POSITIVE_INFINITY);
  motorR.setPosition(Double.POSITIVE_INFINITY);
  motorL.setVelocity(0);
  motorR.setVelocity(0);
  suppressed = true;
  
}


public boolean takeControl() {
  if (callStep) { 
    robot.step(32);
    } 
  
 // System.out.println("usDerecho.value = " + sensorValue[2]);
  if (working) {return true;}
  else {double[] lectures = robot.getIRValues();
  for (int i = 0; i < lectures.length; i++) {  
    if (lectures[i] > radius[i]) {
      sensorValues = lectures.clone();
      System.out.println("Avoid2 takeControl true");

      return true;
    }
  }
  System.out.println("Avoid2 takeControl false");
  return false;
  }
}  

public void action() {
  suppressed = false;
  working = true;
  while (!suppressed) {
  //calculo la fuerza atractora: el nido
  double[] myPos = robot.currentPosition();
  nestDir = Math.atan2(robot.nestCoords[1]- myPos[1], robot.nestCoords[0]- myPos[0] );
  double nestDist = Math.hypot(robot.nestCoords[0]- myPos[0], robot.nestCoords[1]- myPos[1]);
  //double currentDir = robot.currentDirection();
  theta = robot.turningAngle(robot.currentDirection() , nestDir);
  double[] force = {Math.cos(theta)*nestDist , Math.sin(theta)*nestDist };
  //Para cada sensor, si detecta dentro de la burbuja
  //calculo el vector repulsor y lo sumo a fuerza
  for (int i = 0; i < sensorValues.length; i++) {  
    if (sensorValues[i] > radius[i]) {
    // el vector repulsor tiene direccion repulsionDir[i]
    // y modulo  1: normalio el valor del sensor
    //force[0] += Math.cos(repulsionDir[i])* sensorValues[i]/1023;
    //force[1] += Math.sin(repulsionDir[i])* sensorValues[i]/1023;
    force[0] += Math.cos(repulsionDir[i])* sensorValues[i];
    force[1] += Math.sin(repulsionDir[i])* sensorValues[i];
    
    } 
  }
  
  //ajusto W y V segun la fuerza resultante 
  theta = Math.atan2( force[1], force[0] );
  //System.out.println("girar " + Math.toDegrees(theta));
  System.out.println("fuerza " + force[0] + ", "+ force[1] );
  W = Math.signum(theta)* Math.PI;
  V = 0.05;
  wr = robot.computewr(V ,W);
  wl = robot.computewl( V,W);
  motorL.setVelocity(wl);
  motorR.setVelocity(wr);
  working = false;
 }// fin del while
  }

public void suppress() {
  System.out.println("Avoid2 suppress");
  suppressed = true;
  }

}