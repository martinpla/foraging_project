// File:          SupervisorController.java
// Date:
// Description:
// Author:
// Modifications:

// You may need to add other webots classes such as
//  import com.cyberbotics.webots.controller.DistanceSensor;
//  import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Supervisor;
import com.cyberbotics.webots.controller.Node;
import com.cyberbotics.webots.controller.Field;
// Here is the main class of your controller.
// This class defines how to initialize and how to run your controller.
public class SupervisorController {

  // This is the main function of your controller.
  // It creates an instance of your Robot instance and
  // it uses its function(s).
  // Note that only one instance of Robot should be created in
  // a controller program.
  // The arguments of the main function can be specified by the
  // "controllerArgs" field of the Robot node
  public static void main(String[] args) {

    // create the Robot instance.
    Supervisor sup = new Supervisor();
    
    // Creo la fuente de comida si no esta creada aun
    Node FS = sup.getFromDef("FS");
    if(FS == null) { 
      Node root = sup.getRoot();
      Field root_children = root.getField("children");
      root_children.importMFNodeFromString(-1, "DEF FS FoodSource {}");
    };

    // get the time step of the current world.
    //int timeStep = (int) Math.round(sup.getBasicTimeStep());

    // You should insert a getDevice-like function in order to get the
    // instance of a device of the robot. Something like:
    //  Motor motor = robot.getMotor("motorname");
    //  DistanceSensor ds = robot.getDistanceSensor("dsname");
    //  ds.enable(timeStep);

    // Main loop:
    // - perform simulation steps until Webots is stopping the controller
    boolean terminate = false;
    while ( sup.step(32) != -1 ) {
    };

    // Enter here exit cleanup code.
  }
}
