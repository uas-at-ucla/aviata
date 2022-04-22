#ifndef GPIO_H_
#define GPIO_H_

#define OUTPUT_PIN 26
#define INPUT_PIN 16
#define SERVO_PIN 25
#define SERVO_PWM 64

class DockingDetector
{

public:
    DockingDetector();
    ~DockingDetector();
    bool is_docked();
    bool disengage_servo();
    bool engage_servo();

private:
    bool _is_sim;
    bool _gpio_initialized;

    bool output_signal(unsigned signal);
    int read_signal();  
};

#endif // DRONE_H_