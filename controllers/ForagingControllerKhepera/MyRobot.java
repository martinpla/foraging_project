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
 private final double cellSize = 0.1;
 public int[] currentCell = new int[2]; 
 
 public double[] atractor; 
 public boolean atractorIsObject;
 
 public Robot robot;
 public GPS gps;
 public Compass compass;
 public Motor motorL, motorR;
 public DistanceSensor usII, usI, usF, usD, usDD;
 //thread for producing pheromone
 static ScheduledExecutorService produceThread;  
   
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

//Creo el nido si no esta creada ya
Node nest = this.getFromDef("nest");
if(nest == null) { 
  root_children.importMFNodeFromString(-1, "DEF nest Nest {}");
  
  nest = this.getFromDef("nest");
  //Field nestTranslationField = nest.getField("translation");
  double[] nestTranslation = nest.getField("translation").getSFVec3f(); 
  nestCoords = new double[2];
  nestCoords[0] = nestTranslation[0]; 
  nestCoords[1] = -nestTranslation[2]; 
  
  nestRadius = nest.getField("radius").getSFFloat();
  } 

//Creo la fuente de comida si no esta creada ya
Node fs = this.getFromDef("fs");
if(fs == null) { 
  root_children.importMFNodeFromString(-1, "DEF fs FoodSource {}");
  } 

//Creo el grid si no esta creado ya
Node grid = this.getFromDef("grid");
if(grid == null) { 
  root_children.importMFNodeFromString(-1, "DEF grid Group {}");
  } 

//

/*
//creo el hilo encargado de dejar feromona
produceThread = Executors.newSingleThreadScheduledExecutor();
//Tdelay es el cada cuanto chequeo si cambie de celda
//tdelay debe ser menor que cellSize/Vmax 
long Tdelay = (long) ( ( cellSize*1000 ) / computeAbsV(0))  ;
produceThread.scheduleWithFixedDelay(new Runnable() {
            @Override
            public void run() {
              //me fijo si current cell cambió
              double[] pos = currentPosition();
              int[] cell = { (int) Math.round( pos[0]/cellSize), (int) Math.round( pos[1]/cellSize)  };
              if( ! Arrays.equals(cell, currentCell)) {
                //si currentCell es distinto de mi celda actual
                //significa que entré a una nueva celda. Actualizo currentCell
                currentCell = cell.clone();
                
                // Si vengo con comida dejo dos feromonas en la nueva celda
                //if (hasFood)  { releasePheromone(2); } 
            }
        }, 0, Tdelay /2, TimeUnit.MILLISECONDS);

*/
//this.releasePheromone();
} //fin constructor

public void releasePheromone(int quantity) {
//leaves two particles of pheromone at the current location cell 
double[] pos = currentPosition();
int x = (int) Math.round( pos[0]/cellSize);
int y = (int) Math.round( pos[1]/cellSize);
Node cell = this.getFromDef(String.format("cell_%d_%d", x, y));
if(cell == null) { createCell(x,y); 
 } else {
  Field f = cell.getField("value");
  int newValue = f.getSFInt32() + quantity;
  f.setSFInt32(newValue + quantity);
  }
}
 
public void createCell(int x, int y) {
Field grid_children = this.getFromDef("Grid").getField("children");
String Cell_str = String.format("DEF cell_%d_%d Cell{%n", x, y);
Cell_str = Cell_str.concat("  translation " + x*cellSize  + " 0.01 " + -1*y*cellSize  );
String size_str = "  cellSize " + cellSize + " " + cellSize ;
Cell_str = Cell_str.concat(size_str);
Cell_str = Cell_str.concat("}");
grid_children.importMFNodeFromString(-1, Cell_str); 
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
  System.out.println("getSensorValues");
  return values;
  } 
 
/* 
  public double[] getIRValues() {
  double[] values = new double[4]; 
  values[0] = dsI.getValue();
  values[1] = dsFI.getValue();
  values[2] = dsFD.getValue();
  values[3] = dsD.getValue();
  return values;
  }
*/   
}