#include "pigpio.h"
#include <iostream>
#include <stdlib.h>

int main(int argc, char*[]argv){
    if(argc != 2){
        std::cout << "One argument required: pwm cycle for servo" << std::endl;
        return 1;
    }
    int deg = atoi(argv[1]);

    gpioInitialise();
    gpioSetMode(25, PI_OUTPUT);
    gpioPWM(25, deg);
    gpioTerminate();
    return 0;
}