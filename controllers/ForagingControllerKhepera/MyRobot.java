import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.GPS;
import com.cyberbotics.webots.controller.Compass;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Supervisor;
import com.cyberbotics.webots.controller.Node;
import com.cyberbotics.webots.controller.Field;
import java.util.Arrays;
import java.util.Date;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class MyRobot extends Supervisor {
 
 final double R = 0.0212;//wheel radius, value at line 2266 del proto file o 20,5 mm segun el user manual
 final double L = 0.09156; // distance between wheels, valor obtenido restando los translation de cada rueda en el proto file
 
 // V and W are lineal and angular velocities of the robot
 // this constraint must hold a|V| + b|W| <= wmax
 public double wmax = 17; // maximum angular velocity of the wheels
 public double a = 1/R;
 public double b = L/(2*R);
 
 public double Vmin = 0.01 ;
 
  
 public double Wmax = computeAbsW(Vmin);
 public double Vmax = computeAbsV(0);
 
 public boolean hasFood = false;
 public int timeStep;
 public double[] nestCoords;
 public double nestRadius;
 public final double cellSize = 0.1;
 public int[] currentCell = new int[2]; 
 public int[] lastMarkedCell = new int[2];
 public double[] atractor; 
 public boolean atractorIsObject;
 public Field grid_children;
 //public Field root_children;
 public Robot robot;
 public GPS gps;
 public Compass compass;
 public Motor motorL, motorR;
 public DistanceSensor usII, usI, usF, usD, usDD;
  
 public MyRobot() {
 //constructor

 timeStep = (int) Math.round(this.getBasicTimeStep());
 gps = this.getGPS("gps");
 gps.enable(timeStep);
 compass = this.getCompass("compass");
 compass.enable(timeStep);
 
 usII = this.getDistanceSensor("us0");
 usI = this.getDistanceSensor("us1");
 usF = this.getDistanceSensor("us2");
 usD = this.getDistanceSensor("us3");
 usDD = this.getDistanceSensor("us4");
 
 usI.enable(timeStep);
 usF.enable(timeStep);
 usD.enable(timeStep);
 usDD.enable(timeStep);
 usII.enable(timeStep);
 
 this.step(timeStep);
 motorL = this.getMotor("left wheel motor"); 
 motorR = this.getMotor("right wheel motor");
 motorL.setPosition(Double.POSITIVE_INFINITY);
 motorR.setPosition(Double.POSITIVE_INFINITY); 
 motorL.setVelocity(0);
 motorR.setVelocity(0);
  
 //asigno currentCell
double[] pos = currentPosition();
currentCell[0] = (int) Math.round( pos[0]/cellSize);
currentCell[1] = (int) Math.round( pos[1]/cellSize);

atractor = null;
atractorIsObject = false;

Node root = this.getRoot();
Field root_children = root.getField("children");

//Create nest if it doesn´t exists already
Node nest = this.getFromDef("nest");
if(nest == null) { 
  root_children.importMFNodeFromString(-1, "DEF nest Nest {}");
  
  nest = this.getFromDef("nest");
  Field nestTranslationField = nest.getField("translation");
  double[] nestTranslation = nestTranslationField.getSFVec3f(); 
  nestCoords = new double[2];
  nestCoords[0] = nestTranslation[0]; 
  nestCoords[1] = -nestTranslation[2];  
  nestRadius = nest.getField("radius").getSFFloat();
  } 

//Create food source if it doesn´t exists already
Node fs = this.getFromDef("fs");
if(fs == null) { 
  root_children.importMFNodeFromString(-1, "DEF fs FoodSource {}");
  } 

//Create grid if it doesn´t exists already
Node grid = this.getFromDef("grid");
if(grid == null) { 
  root_children.importMFNodeFromString(-1, "DEF grid Group {}");
  grid = this.getFromDef("grid");
  grid_children = grid.getField("children");
  } 
} //fin constructor

public int pheromoneValue(int x, int y) {
//Look for cell x y
String cellName = String.format("cell_%d_%d", x, y);
Node cell = this.getFromDef(cellName);
if(cell == null) { return 0;}
else { Field f = cell.getField("value");
       if(f == null) { return 0;} 
       else { int value = f.getSFInt32(); 
              return value; } 
       }
}

public void releasePheromone(int quantity) {
//leaves particles of pheromone at the current cell 
double[] pos = currentPosition();
int x = (int) Math.round( pos[0]/cellSize);
int y = (int) Math.round( pos[1]/cellSize);
String cellName = String.format("cell_%d_%d", x, y);
Node cell = this.getFromDef(cellName);
if(cell == null) { createCell(x,y); 
 } else {
  Field f = cell.getField("value");
  int newValue = f.getSFInt32();
  newValue += quantity;
  f.setSFInt32(newValue);
  }
}
 
public void createCell(int x, int y) {
String Cell_str = String.format("DEF cell_%d_%d Cell{%n", x, y);
Cell_str = Cell_str.concat("  translation " + x*cellSize  + " 0.01 " + -1*y*cellSize  );
String size_str = "  cellSize " + cellSize + " " + cellSize ;
Cell_str = Cell_str.concat(size_str);
Cell_str = Cell_str.concat("}");
Node root = this.getRoot();
Field root_children = root.getField("children");
root_children.importMFNodeFromString(-1, Cell_str); 
}

 public double computeAbsV(double W) {
   return (wmax - b*Math.abs(W)) / a;
 }
 
 public double computeAbsW(double V) {
   return (wmax - a*V) / b;
 }

 public double computewr(double V,double W) {
   return a*V + b*W;
 }

 public double computewl(double V, double W) {
   return a*V - b*W;
 }
 
 public double currentDirection() {
   double[] dir = compass.getValues();
   double radians = Math.atan2(dir[2],dir[0]) ;
   return radians;
   }
   
 public double[] currentPosition() {
   double[] pos = gps.getValues();
   double[] out = { pos[0], -pos[2] };
   return out;
   }
  
   
 public double turningAngle(double ini, double fin) {
   // receives initial and final orientations
   // return theta = best turning angle between -PI and +PI 
   // from initial orientation to reach final orientation 
   double theta;
   theta = fin - ini;
   if (Math.abs(theta) <= Math.PI) 
     {return theta;}
   else {
    return - Math.signum(theta)* ( 2*Math.PI - Math.abs(theta) ); 
    } 
   }
   
 public double[] getSensorValues() {
  double[] values = new double[5]; 
  values[0] = usII.getValue();
  values[1] = usI.getValue();
  values[2] = usF.getValue();
  values[3] = usD.getValue();
  values[4] = usDD.getValue();
  return values;
  } 
 
 }