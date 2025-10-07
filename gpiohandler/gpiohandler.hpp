#include <iostream>
#include <gpiod.h>

class GpioHandler {
public:
    GpioHandler(std::string lamp_pattern = "R") {
        gpio_chip = gpiod_chip_open_by_name("gpiochip0");
        if (gpio_chip == nullptr) {
            return;
        }
        red_line = gpiod_chip_get_line(gpio_chip, 12);
        green_line = gpiod_chip_get_line(gpio_chip, 13);
        blue_line = gpiod_chip_get_line(gpio_chip, 18);
        gpiod_line_request_output(red_line, "red", 0);
        gpiod_line_request_output(green_line, "green", 0);
        gpiod_line_request_output(blue_line, "blue", 0);
        // Parse lamp_pattern into a vector of strings, delimited by ','
        std::vector<std::string> lamp_pattern_vec;
        size_t start = 0, end = 0;
        while ((end = lamp_pattern.find(',', start)) != std::string::npos) {
            lamp_pattern_vec.push_back(lamp_pattern.substr(start, end - start));
            start = end + 1;
        }
        lamp_pattern_vec.push_back(lamp_pattern.substr(start));
        lamp_pattern_index = 0;
    }

    ~GpioHandler() {
        closeGpio();
    }

    void setNextLampColor() {
        bool wasColorSet = false;
        if (lamp_pattern_index == lamp_pattern_vec.size()) {
            lamp_pattern_index = 0;
        }
        std::string current_color = lamp_pattern_vec[lamp_pattern_index];
        setRedLow();
        setGreenLow();
        setBlueLow();
        for (auto letter : current_color) {
            if (letter == 'R') {
                setRedHigh();
                wasColorSet = true;
            } else if (letter == 'G') {
                setGreenHigh();
                wasColorSet = true;
            } else if (letter == 'B') {
                setBlueHigh();
                wasColorSet = true;
            } else if (letter == 'W') {
                setRedHigh();
                setGreenHigh();
                setBlueHigh();
                wasColorSet = true;
            }
        }
        if (!wasColorSet) {
            setRedHigh();
        }
        lamp_pattern_index++;
        if (lamp_pattern_index == lamp_pattern_vec.size()) {
            lamp_pattern_index = 0;
        }
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
        setRedLow();
        setGreenLow();
        setBlueLow();
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
    std::vector<std::string> lamp_pattern_vec;
    int lamp_pattern_index;
};