#include "MyGpio.h"
#include <unistd.h>  // For sleep()

int main() {
    MyGpio button("gpiochip0", 17);  //ACTIVATE BUTTON

    if (!button.init()) {
        std::cerr << "Failed to initialize GPIO button" << std::endl;
        return 1;
    }

    MyGpio button2("gpiochip0", 27); //EXTEND BUTTON

    if (!button2.init()) {
        std::cerr << "Failed to initialize GPIO button" << std::endl;
        return 1;
    }

    MyGpio button3("gpiochip0", 24);  //HOME BUTTON

    if (!button3.init()) {
        std::cerr << "Failed to initialize GPIO button" << std::endl;
        return 1;
    }


    while (true) {
        //button1
        int value = button.readValue();

        if (value == -1) {
            std::cerr << "Error reading GPIO value" << std::endl;
            break;
        }

        if (value == 0) {
            std::cout << "Button pressed11111111111111111111111111111111111111!" << std::endl;
        } else {
            std::cout << "Button not pressed." << std::endl;
        }

        //button2
        int value2 = button2.readValue();

        if (value2 == -1) {
            std::cerr << "Error reading GPIO value" << std::endl;
            break;
        }

        if (value2 == 0) {
            std::cout << "Button pressed222222222222222222222222222222222222222222222!" << std::endl;
        } else {
            std::cout << "Button not pressed." << std::endl;
        }

        //button3
        int value3 = button3.readValue();

        if (value3 == -1) {
            std::cerr << "Error reading GPIO value" << std::endl;
            break;
        }

        if (value3 == 0) {
            std::cout << "Button pressed3333333333333333333333333333333333!" << std::endl;
        } else {
            std::cout << "Button not pressed." << std::endl;
        }

       // sleep(1); // Check the button state every second
    }

    return 0;
}
