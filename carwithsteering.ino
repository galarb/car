#include "car.h"
#include "clicli.h"
//Reflectedligh Pin on A1.  
Car mycar(
  5, //servoPin
  7, //encoderPin
  3, //Ena on L298N
  2, //in1
  4, //in2
  80); //wheel diameter
clicli mycli(mycar);
void setup() {
  mycar.begin(115200);  
  
}

void loop() {
 mycli.run();
 //mycar.trackline(100, 34, 20, 0, 0);
}
