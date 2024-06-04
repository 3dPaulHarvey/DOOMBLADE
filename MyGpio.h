#ifndef MYGPIO_H
#define MYGPIO_H

#include <gpiod.h>
#include <string>
#include <iostream>

class MyGpio {
public:
    MyGpio(const std::string& chipname, unsigned int gpio)
        : chipname(chipname), gpio(gpio), line(nullptr), chip(nullptr) {}

    ~MyGpio() {
        if (line) {
            gpiod_line_release(line);
        }
        if (chip) {
            gpiod_chip_close(chip);
        }
    }

    bool init() {
        chip = gpiod_chip_open_by_name(chipname.c_str());
        if (!chip) {
            std::cerr << "Error opening GPIO chip: " << chipname << std::endl;
            return false;
        }

        line = gpiod_chip_get_line(chip, gpio);
        if (!line) {
            std::cerr << "Error getting GPIO line: " << gpio << std::endl;
            gpiod_chip_close(chip);
            return false;
        }

        gpiod_line_request_config config;
        config.consumer = "button_test";
        config.request_type = GPIOD_LINE_REQUEST_DIRECTION_INPUT;
        config.flags = GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP;

        if (gpiod_line_request(line, &config, 0) != 0) {
            std::cerr << "Failed to request line as input" << std::endl;
            return false;
        }

        return true;
    }

    int readValue() {
        if (!line) return -1; // Ensure line is valid
        return gpiod_line_get_value(line);
    }

private:
    std::string chipname;
    unsigned int gpio;
    gpiod_line* line;
    gpiod_chip* chip;
};

#endif // MYGPIO_H
