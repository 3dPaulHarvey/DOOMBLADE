#include <iostream>
#include <chrono>
#include <deque> // For maintaining a rolling window of times
#include <cmath>
#include <ctime>
#include <unistd.h> // For nanosleep
#include <limits>
#include "MyController.h"



int main() {
    // CONTROLLER SETUP
    MyController controller("/dev/fdcanusb");
    if (!controller.setupSerialPort()) {
        std::cerr << "Failed to setup serial port" << std::endl;
        return 1;
    }
    controller.sendStopCommand();  //gets controller to a known state
    controller.sendRezeroCommand(500.0f); // sets the current position to 500.0

    struct timespec req = {0, 1200 * 1000};

    std::deque<std::chrono::microseconds> times; // Store last 10 times
    const size_t maxTimes = 10; // Maximum number of times to average

    while (true) {
        auto start = std::chrono::high_resolution_clock::now();

        // Run the provided code snippet
        controller.sendWriteCommand(std::numeric_limits<float>::quiet_NaN(), 0.0);
        nanosleep(&req, NULL); // Sleep for req time
        auto controller_state = controller.sendReadCommand();
        
        if (controller_state[0] >= 450 && controller_state[0] <= 550) {
            //print the first three elements of controller state


        }

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

        // Manage the rolling window of times
        if (times.size() >= maxTimes) {
            times.pop_front(); // Remove the oldest time
        }
        times.push_back(duration); // Add the new time

        // Calculate the average of the times
        std::chrono::microseconds sum(0);
        for (auto t : times) {
            sum += t;
        }
        auto averageTime = sum.count() / times.size();

        std::cout << controller_state[0] << "\t " << averageTime << " micro." << std::endl;
        // std::cout << controller_state[0] << " " << controller_state[1] << " " << controller_state[2] << "Ave  " << times.size() 
        //           << " " << averageTime << " micro." << std::endl;
    }

    return 0;
}
