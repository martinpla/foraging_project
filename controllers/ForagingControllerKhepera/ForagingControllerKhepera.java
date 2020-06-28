// File:          ForagingControllerKhepera.java
// Date:
// Description:
// Author:
// Modifications:

// You may need to add other webots classes such as
//  import com.cyberbotics.webots.controller.DistanceSensor;
//  import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;
import lejos.robotics.subsumption.*;
import java.lang.Thread;

import com.cyberbotics.webots.controller.Supervisor;
import com.cyberbotics.webots.controller.Node;
import com.cyberbotics.webots.controller.Field;
// Here is the main class of your controller.
// This class defines how to initialize and how to run your controller.
public class ForagingControllerKhepera {

  // This is the main function of your controller.
  // It creates an instance of your Robot instance and
  // it uses its function(s).
  // Note that only one instance of Robot should be created in
  // a controller program.
  // The arguments of the main function can be specified by the
  // "controllerArgs" field of the Robot node
  public static void main(String[] args) {
    MyRobot robot = new MyRobot();
    

    //comportaminetos por orden de mayor prioridad
    // El segundo argumento del metodo constructor indica 
    //si debe llamar al stepo de la simulacion
    // Solo el de mayor prioridad lleva este argumento en true
    
    //MyBehavior B = new MyBehavior(robot);    
    Avoid avoid = new Avoid(robot, true);
    UnloadFood unload = new UnloadFood(robot, false);
    LoadFood load = new LoadFood(robot, false);
    GoToNest goToNest = new GoToNest(robot, false);
    GoToFood goToFood = new GoToFood(robot, false);
    Wander wander = new Wander(robot, false, 7);
    //Behavior[] Behaviors = {goToNest,  avoid};
    Behavior[] Behaviors = {wander, goToFood, goToNest, load, unload, avoid};
    Arbitrator A = new Arbitrator(Behaviors);
    A.start();    

  }
}
