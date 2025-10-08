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
        LOG(1, "Lamp pattern: " << lamp_pattern);
        size_t start = 0, end = 0;
        while ((end = lamp_pattern.find(',', start)) != std::string::npos) {
            lamp_pattern_vec.push_back(lamp_pattern.substr(start, end - start));
            start = end + 1;
        }
        lamp_pattern_vec.push_back(lamp_pattern.substr(start));
        lamp_pattern_index = 0;
        LOG(1, "Lamp pattern vector: ");
        for (auto color : lamp_pattern_vec) {
            LOG(1, color);
        }
    }

    ~GpioHandler() {
        closeGpio();
    }

    void setNextLampColor() {
        LOG(1, "Setting next lamp color");
        bool wasColorSet = false;
        bool setRed = false;
        bool setGreen = false;
        bool setBlue = false;
        if (lamp_pattern_index == lamp_pattern_vec.size()) {
            lamp_pattern_index = 0;
        }
        // LOG(1, "Vector size: " << lamp_pattern_vec.size());
        // LOG(1, "Lamp pattern index: " << lamp_pattern_index);
        std::string current_color = lamp_pattern_vec[lamp_pattern_index];
        LOG(1, "Current color: " << current_color);
        LOG(1, "Color size: " << current_color.size());
        for (unsigned int i = 0; i < current_color.size(); i++) {
            char letter = current_color[i];
            // LOG(1, "Letter: " << letter);
            // LOG(1, "Current color: " << current_color);
            // LOG(1, "Lamp pattern index: " << lamp_pattern_index);
            // LOG(1, "Lamp pattern vector: ");
            for (auto color : lamp_pattern_vec) {
                LOG(1, color);
            }
            if (letter == 'R') {
                setRed = true;
                wasColorSet = true;
            } else if (letter == 'G') {
                setGreen = true;
                wasColorSet = true;
            } else if (letter == 'B') {
                setBlue = true;
                wasColorSet = true;
            } else if (letter == 'W') {
                setRed = true;
                setGreen = true;
                setBlue = true;
                wasColorSet = true;
            }
        }
        if (!wasColorSet) {
            setRed = true;
        }
        if (setRed) {
            setRedHigh();
        } else {
            setRedLow();
        }
        if (setGreen) {
            setGreenHigh();
        } else {
            setGreenLow();
        }
        if (setBlue) {
            setBlueHigh();
        } else {
            setBlueLow();
        }
        lamp_pattern_index++;
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
    unsigned int lamp_pattern_index;
};