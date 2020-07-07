import com.cyberbotics.webots.controller.Robot;
import lejos.robotics.subsumption.*;
import com.cyberbotics.webots.controller.Supervisor;
import com.cyberbotics.webots.controller.Node;
import com.cyberbotics.webots.controller.Field;
// Here is the main class of your controller.
// This class defines how to initialize and how to run your controller.
public class ForagingControllerKhepera {

  public static void main(String[] args) {
    MyRobot robot = new MyRobot();
    //comportaminetos por orden de mayor prioridad
    // El segundo argumento del metodo constructor indica si debe llamar al step de la simulacion
    // Solo el de mayor prioridad lleva este argumento en true    
    Avoid avoid = new Avoid(robot, true);
    UnloadFood unload = new UnloadFood(robot, false);
    LoadFood load = new LoadFood(robot, false);
    GoToNest goToNest = new GoToNest(robot, false);
    GoToFood goToFood = new GoToFood(robot, false);
    FollowTrail followTrail = new FollowTrail(robot, false);
    Wander wander = new Wander(robot, false, 7);
    Behavior[] Behaviors = {wander, followTrail, goToFood, goToNest, load, unload, avoid};
    Arbitrator A = new Arbitrator(Behaviors);
    A.start();    
  }
}
