import java.util.Arrays;
import lejos.robotics.subsumption.*;
//  import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Node;
import com.cyberbotics.webots.controller.Field;

public class FollowTrail implements Behavior {

private MyRobot robot;
int timeStep;

private Motor motorR;  
private Motor motorL;

private boolean suppressed;
private boolean callStep;
private double cellSize;

double[] myPos;
 //my cell index

//private GPS gps
public FollowTrail(MyRobot r, boolean cs) {
  robot = r;
  motorR = r.getMotor("right wheel motor");
  motorL = r.getMotor("left wheel motor");
  
  suppressed = true;
  callStep = cs;
  cellSize = robot.cellSize;
   
}

public boolean takeControl() {
  if (callStep) { 
    robot.step(32);
    } 
  String cellName;
  //devulve true si detecta rastro
  // detectar rastro podria significar que 
  // mi celda actual tiene peromona o alguna de las adyacentes
  // serìa la minima condicion de deteccion de feromona
  double[] myPos = robot.currentPosition();
  int x = (int) Math.round( myPos[0]/cellSize);
  int y = (int) Math.round( myPos[1]/cellSize);
  int value;
  
  for(int i = x - 1; i <= x + 1 ; i++ ) {
    for(int j = y - 1; j <= y + 1 ; j++ ) {
      value = robot.pheromoneValue(i,j);
       if(value > 0) {return true;}
      }
   }
  return false; 
}

public void action() {
  
  int x,y, i, j;
  double P, v , d, destDir, theta , V, W, wr, wl, random;
  V = robot.Vmin;
  //probabilities matrix
  double[][] probs = new double[3][3];
  
  
  suppressed = false;
  while (!suppressed) {
  //a cada celda adyacente le adjudico cierta probalidad p
  //p aumentará respecto a la cantidad de feromona en la celda v
  // y disminuira respecto a la distancia a la celda actual (distancia como norma manhatan )
  // es decir  
  //x y = celda actual
  double[] myPos = robot.currentPosition();
  x = (int) Math.round( myPos[0]/cellSize);
  y = (int) Math.round( myPos[1]/cellSize);
  System.out.println("Follow Action en la celda " + x + " " + y);

  
  P = 0; 
  v = 0; 
  d = 0;
  for( i = 0; i <= 2 ; i++ ) {
    for( j = 0; j <= 2 ; j++ ) {
      //para cada celda adyacente obtengo su value v
      //me fijo si existe y si tiene value 
      //si no existe o value = 0 asigno v pequeño v = 0.5 por ejemplo 
      if( i == 1 && j == 1 )
        { probs[i][j] = 0; d= 0; }   
      else
        {
        // distancia a la celda x y     
        d = (double) ( Math.abs(i - 1) + Math.abs( j - 1)); 
        //me fijo si tiene feromona
        v = (double) robot.pheromoneValue(x-1+i, y-1+j);
        if(v <= 0) { v = 0.2;}
          probs[i][j] =  v / d ;
        }    
      P += probs[i][j]; 
       }
  } // fin primer recorrida matriz> inicializacion
  System.out.println("bandera 1");
  i = 0;
  j = 0;
  //sorteo numero al azar entre 0 y P
  random = Math.random()*P;
  //recorro la matriz y veo en el segmento de que celda cayo el numero sorteado
  OUTTER_LOOP:
    for( i = 0; i <= 2 ; i++ ) {
      for( j = 0; j <= 2 ; j++ ) {
        //para cada celda adyacente obtengo su value v
        //me fijo si existe y si tiene value 
        //si no existe o value = 0 asigno v pequeño v = 0.5 por ejemplo 
       if( random <= probs[i][j] )
          { break OUTTER_LOOP;  }
        else 
          {  random -= probs[i][j]; } 
        }   
     }
   System.out.println("bandera 2");

   //asigno a i j como celda destino
   i += x-1 ;
   j += y-1 ;  
   System.out.println(String.format("celda actual %d %d, celda elegida: %d %d", x, y, i, j));
  //x y = celda actual
  while(x != i || y != j) {
    // mientras no este en la nueva celda
    //calculo el angulo hacia la celda de destino i j
    myPos = robot.currentPosition();
    destDir = Math.atan2( j*cellSize - myPos[1], i*cellSize - myPos[0]) ;
    theta = robot.turningAngle(robot.currentDirection(), destDir);
    W = Math.signum(theta)*theta*theta*(robot.Wmax / (Math.PI*Math.PI));
    //V = robot.computeAbsV(W);
    wl = robot.computewl(V,W);
    wr = robot.computewr(V,W);
    motorL.setVelocity(wl);
    motorR.setVelocity(wr);
    
    // actualizo celda actual 
    x = (int) Math.round( myPos[0]/cellSize);
    y = (int) Math.round( myPos[1]/cellSize);
  }   
  
  //si sali del while es porque llegue a la nueva celda
  //si en la nueva celda hay feromona dejo 1, si no hay dejo 2 
  /*
  if( robot.pheromoneValue(x,y) > 0 ) { robot.releasePheromone(1);}
  else { robot.releasePheromone(2);}
  */
  
  } //fin while

} //fin action 
  /*double[] myPos;
  int[] cell = new int[2];
  double nestDir, theta , V, W, wr, wl;
  suppressed = false;
  robot.atractor = robot.nestCoords.clone();
  robot.atractorIsObject = true;
  while (!suppressed) {
    
     myPos = robot.currentPosition();
     nestDir = Math.atan2(robot.nestCoords[1]- myPos[1], robot.nestCoords[0]- myPos[0] );
     double currentDir = robot.currentDirection();
     theta = robot.turningAngle(currentDir, nestDir);
     
     
     W = Math.signum(theta)*theta*theta*(robot.Wmax / (Math.PI*Math.PI));
     V = robot.computeAbsV(W);
     
     wl = robot.computewl(V,W);
     wr = robot.computewr(V,W);
     motorL.setVelocity(wl);
     motorR.setVelocity(wr);
     
     //Si la celda actual es distinta de la ultima marcada con feromona
     //dejo 2 feromonas
     //obtengo celda actual
     //double[] pos = currentPosition(); // current position coords are rounded to Int
     //cell = { (int) Math.round( myPos[0]/cellSize), (int) Math.round( myPos[1]/cellSize)  };
     cell[0] = (int) Math.round( myPos[0]/cellSize);
     cell[1] = (int) Math.round( myPos[1]/cellSize);
     
     
     if( ! Arrays.equals(cell, robot.lastMarkedCell)) {
                //si currentCell es distinto de mi celda actual
                //significa que entré a una nueva celda. Actualizo currentCell
                robot.lastMarkedCell = cell.clone();
                
                
                // Si vengo con comida significa que el ultimo activo fue goToNest
                // en ese caso dejo dos feromonas en la nueva celda
                if (robot.hasFood)  { robot.releasePheromone(2); }
                
                // else TO DO 
                // faltan condiciones para follow trail 
                //se 
                */
      
  

public void suppress() {
  suppressed = true;
  
  }

}