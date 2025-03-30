#include "sensors.h"
#include <iostream>


int main(){
    SensorController<ButtonController> sensorController(ButtonController("/dev/uio4", "/dev/uio5"));
    SensorData sd;
    while(1){
        sensorController.handle(sd);
        if(sd.leftBoundary){
            std::cout << "Left Boundary Triggered" << std::endl;
        }else if (sd.rightBoundary){
            std::cout << "Right Boundary Triggered" << std::endl;
        
        }
    }
    return 0;
}
