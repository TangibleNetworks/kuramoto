// Kuramoto.ino - 19/05/2014
// Tangible Networks
// Lewis, Chris and Espen
//
// Kuramoto model of synchronising oscillators.  Nodes have natural frequencies settable
// with the DIP switches.  The coupling strength between the nodes is adjusted with
// the master controller.  With sufficient coupling strength (how much is needed depends 
// on the topology) the nodes will synchronise and flash at the same frequency.
//
// Equation is
// theta_dot = omega - k*\sum_{j} sin(theta - theta_j) 
// where j goes over the 3 inputs.  Forward Euler timestepping loop.
//
// Oscillations are shown by the nodes flashing, and the colour is determined from the 
// phase difference to its neighbours.  So if the colours are changing the nodes are 
// not synchronised - when the colours are all the same the network is synchronised.
//
// Worth noting that with '-k' you get phase synchronisation, with '+k' you get phase locking!
//
// New implementation for TN-04, using library.

#include <math.h>
#include <TN.h>

TN Tn = TN(0.0,2.0*PI);

// Model parameters
double dt = 0.05;
double theta = 0.0;
double theta_old;
double omegas[] = {1.0, 2.0}; // range of omega.
double omega = 0.0;
double k[] = {0.0, 1.0}; // this defines the range of k (coupling strength)
double thisK;



// Other variables
double ins[3] = {0.0,0.0,0.0};
double synchronisation = 0;

void setup () {}


void loop() {
  
  omega = omegas[0] + (omegas[1]-omegas[0])*Tn.pot(); // select omega based on pot
  
  // Forward Euler timestepping loop  
  theta_old = theta;
  theta += omega*dt;
  
  if (Tn.masterConnected()) thisK = k[0] + (k[1] - k[0])*Tn.masterRead();
  else thisK = k[0] + 0.5*(k[0] + k[1]);
  // Loop over the inputs.  Ignore them if they're not connected.
  for (int i=0; i<3; i++) {
    if (Tn.isConnected(i+1)) {
      ins[i] = Tn.analogRead(i+1);
      theta -= dt*thisK*sin(theta_old - ins[i]);
    }
  }
  
  // If switch is pressed, we want to ignore all that and just freeze the node.
  if (Tn.sw()) theta = theta_old;  
  
  theta = fmod(theta,2*PI);
  
  Tn.analogWrite(1,theta);
  Tn.analogWrite(2,theta);
  Tn.analogWrite(3,theta);
  
  // We decide the colour based on the average value of the cosine 
  // of the phase difference between each of the inputs.  Need to 
  // take into account if an input isn't connected.
  synchronisation = 0;
  int connections = 0;
  for (int i=0; i<3; i++) {
    if (Tn.isConnected(i+1)) {
      synchronisation = synchronisation + 0.5 + 0.5*cos(theta - ins[i]);
      connections++;
    }
  }
  if (connections>0) {
    synchronisation = synchronisation/connections;
  }
  else { // if there are no connected inputs
    synchronisation = 1;
  }
  
  if (!Tn.sw()) {  // skip if sw pressed
    Tn.colour(0.5*(1.0-synchronisation)*(1+sin(theta)),
    	      0.5*synchronisation*(1+sin(theta)),
    	      0.0);
  }
  // Small delay between iterations.  We cheat a bit, this is not
  // the same as dt so we can vary this to change the speed of the 
  // simulation.
  delay(10);
}

