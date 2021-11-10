#include "docking_detector.hpp"
#include "util.hpp"

#include <string>

#ifdef USE_PIGPIO
#include "pigpio.h"
this doesn't compile whoops // TODO remove once we know this works
#endif

/**
 * Safely read from GPIO to determine docking status
 * 
 * If at any point a GPIO operation fails, safely release the GPIO
 * pins and log the error without terminating the program. This ensures
 * the drone can continue flying (i.e., it'd be unsafe to stop the program
 * while the drone is flying).
 * 
 * This file is only compiled if running on the Raspberry Pi
 * */

DockingDetector::DockingDetector()
{
    std::string tag = "GPIO Initialize";

    #ifdef USE_PIGPIO
    if (gpioInitialise() < 0)
    {
        _gpio_initialized = false;
        log(tag, "Fatal error; proceeding without GPIO functionality", true);
    }
    else
    {
        _gpio_initialized = true;
        log(tag, "Successfully initialized");

        /*
            IMPORTANT
            
            High risk of short-circuit if these pins are not properly initialized

            Current circuit layout:

            -- pin 26 ---->----->--- 330 ohm resistor ---->--------
                                                                | aluminum foil to complete circuit
                                                                | aluminum foil to complete circuit
            -- pin 16 ----<-----<-------------------------<--------

            If the drone is docked, the circuit is completed by the aluminum foil in the
            docking mount. Pin 26 can output a signal and pin 16 will detect it. If pin 16
            does not detect a signal, then the circuit is open and the drone is not docked.

            If the pins are not properly initialized as input and output (e.g. if both are
            outputs), the circuit may short and destroy the board. The 330 ohm resistor in
            series is present to lower this risk, but is not strictly necessary as long as
            the pins are properly initialized.
        */
        int ret1 = gpioSetMode(INPUT_PIN, PI_INPUT);
        int ret2 = gpioSetMode(OUTPUT_PIN, PI_OUTPUT);
        if (ret1 != 0 || ret2 != 0)
        {
            log(tag, "Failed to set input or output pin", true);
            gpioTerminate();
            _gpio_initialized = false;
            return;
        }
        output_signal(0);
    }
    #endif
}

DockingDetector::~DockingDetector()
{
    #ifdef USE_GPIO
    gpioTerminate();
    #endif
}

/**
 * Write a value to the output pin (pin #26)
 * 
 * @param signal is the value to write (HIGH/1 or LOW/0)
 * @return true if successful, false if an error occurred
 * */
bool DockingDetector::output_signal(unsigned signal)
{
    #ifdef USE_PIGPIO
    if (_gpio_initialized)
    {
        if (gpioWrite(OUTPUT_PIN, signal) != 0)
        {
            log("GPIO", "Failed to write 1 to output pin", true);
            gpioTerminate();
            _gpio_initialized = false;
            return false;
        }
        else
        {
            return true;
        }
    }
    #endif

    return false;
}

/**
 * Read from the input pin (pin #16)
 * 
 * @return -1 if an error occurred, or the value that was read (HIGH/1 or LOW/0)
 * */
int DockingDetector::read_signal()
{
    #ifdef USE_GPIO
    if (_gpio_initialized)
    {
        int val = gpioRead(INPUT_PIN);
        if (val != 0 && val != 1)
        {
            log("GPIO", "Failed to read from input pin", true);
            gpioTerminate();
            _gpio_initialized = false;
            return -1;
        }
        else
        {
            return val;
        }
    }
    #endif

    return false;
}

/**
 * Determine if the drone is docked
 * 
 * @return true if docked, false otherwise
 * */
bool DockingDetector::is_docked()
{
    if (_gpio_initialized)
    {
        if (!output_signal(1))
            return false;

        int val = read_signal();
        if (val == -1)
            return false;

        output_signal(0);

        return val == 1;
    }
    else
    {
        log("GPIO", "Unable to determine docking status due to improperly initialized GPIO pins", true);
        return false;
    }
}
