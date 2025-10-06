#include <iostream>
#include <gpiod.h>

class GpioHandler {
public:
    GpioHandler() {
        gpio_chip = gpiod_chip_open_by_name("gpiochip0");
        if (gpio_chip == nullptr) {
            return;
        }
        red_line = gpiod_chip_get_line(gpio_chip, 12);
        green_line = gpiod_chip_get_line(gpio_chip, 13);
        blue_line = gpiod_chip_get_line(gpio_chip, 18);
    }
    ~GpioHandler() {
        gpiod_line_release(red_line);
        gpiod_line_release(green_line);
        gpiod_line_release(blue_line);
        gpiod_chip_close(gpio_chip);
        gpio_chip = nullptr;
    }

    void setRedHigh() {
        gpiod_line_set_value(red_line, 1);
    }
    void setGreenHigh() {
        gpiod_line_set_value(green_line, 1);
    }
    void setBlueHigh() {
        gpiod_line_set_value(blue_line, 1);
    }
    void setRedLow() {
        gpiod_line_set_value(red_line, 0);
    }
    void setGreenLow() {
        gpiod_line_set_value(green_line, 0);
    }
    void setBlueLow() {
        gpiod_line_set_value(blue_line, 0);
    }
    void closeGpio() {
        gpiod_line_release(red_line);
        gpiod_line_release(green_line);
        gpiod_line_release(blue_line);
        gpiod_chip_close(gpio_chip);
        gpio_chip = nullptr;
    }
private:
    gpiod_chip* gpio_chip;
    gpiod_line* red_line;
    gpiod_line* green_line;
    gpiod_line* blue_line;
};