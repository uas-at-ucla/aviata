#ifndef GPIO_H_
#define GPIO_H_

#define OUTPUT_PIN 26
#define INPUT_PIN 16

class DockingDetector
{

public:
    DockingDetector();
    ~DockingDetector();
    bool is_docked();

private:
    bool _is_sim;
    bool _gpio_initialized;

    bool output_signal(unsigned signal);
    int read_signal();
};

#endif // DRONE_H_