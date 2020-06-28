import lejos.robotics.subsumption.*;

public class ArbitratorThread extends Thread {

private Arbitrator A;

public ArbitratorThread(Arbitrator arb) {
A = arb;
}
public void run(){
    System.out.println("ArbitratorThread running");
    A.start();
    }
  }