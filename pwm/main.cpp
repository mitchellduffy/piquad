
#include <stdio.h>
#include "motorcontroller.h"
#include <unistd.h>
#include <iostream>

using namespace std;


int main()
{

  MotorController mc(0.5);

  double v;
  while(true)
  {
    cout << "voltage:  ";
    cin >> v;
    mc.setVoltage(0, v);
    
  }



  return 0;

}
