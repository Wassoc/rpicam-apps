#include <iostream>
#include <gpiod.h>

class GpioHandler {
public:
    GpioHandler() {
        gpio_chip = gpiod_chip_open_by_name("gpiochip0");
        if (gpio_chip == nullptr) {
            return;
        }
    }
    ~GpioHandler() {
        gpiod_chip_close(gpio_chip);
        gpio_chip = nullptr;
    }
    void setGpioHigh(int gpio) {
        gpiod_line_set_value(gpio_chip, gpio, 1);
    }
    void setGpioLow(int gpio) {
        gpiod_line_set_value(gpio_chip, gpio, 0);
    }
    void closeGpio() {
        gpiod_chip_close(gpio_chip);
        gpio_chip = nullptr;
    }
private:
    gpiod_chip* gpio_chip;
};