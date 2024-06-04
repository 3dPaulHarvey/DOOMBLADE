// test_CommandGenerator.cpp
#include <iostream>
#include "CommandGenerator.h"
#include <limits>
#include <chrono>

int main() {
    float testFloat1 = std::numeric_limits<float>::quiet_NaN();
    float testFloat2 = 1.0;

    // Start timing
    auto start = std::chrono::high_resolution_clock::now();

    CommandGenerator generator(testFloat1, testFloat2);
    auto command = generator.getCommand();

    // Stop timing
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    std::cout << "Execution time: " << duration << " microseconds" << std::endl;

    //Print
    std::cout << "Generated Command: ";
    for (auto c : command) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(c) << "";
    }
    std::cout << std::endl;

    std::cout << " Command: ";
    for (auto c : command) {
        std::cout << c << "";
    }
    std::cout << std::endl;

    return 0;
}
