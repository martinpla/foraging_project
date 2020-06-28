import lejos.robotics.subsumption.*;
//  import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;

public class MyBehavior implements Behavior {

private MyRobot robot;
private Motor motorR;  
private Motor motorL;
//private GPS gps

public boolean takeControl() {
  int i = robot.step(32);
  System.out.println("MyBehavior takeControl INICIO");
  System.out.println("DIR actual");
  double dir = robot.currentDirection();
  System.out.println(dir);
  
  System.out.println("MyBehavior takeControl FIN");
  
  return true;
  }

public void action() {
  System.out.println("MyBehavior action");
  }

public void suppress() {
  System.out.println("MyBehavior suppress");
  }

public MyBehavior(MyRobot r) {
  System.out.println("MyBehavior constructor INICIO");
  robot = r;
  motorR = r.getMotor("right wheel motor");
  motorL = r.getMotor("left wheel motor");
  motorL.setPosition(Double.POSITIVE_INFINITY);
  motorR.setPosition(Double.POSITIVE_INFINITY);
  motorL.setVelocity(2*3.14);
  motorR.setVelocity(2*3.14);
  System.out.println("MyBehavior constructor FIN");
  
}

public String palabra() {
return "palabra hermanno";
}

}