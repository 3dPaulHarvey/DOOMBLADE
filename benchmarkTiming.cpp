#include <iostream>
#include <chrono>

int main() {
    // Start timing
    auto start = std::chrono::high_resolution_clock::now();

    // Code to benchmark
    std::cout << "hello" << std::endl;

    // End timing
    auto end = std::chrono::high_resolution_clock::now();

    // Calculate elapsed time in microseconds
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    // Output the time taken to execute the line
    std::cout << "Time taken: " << elapsed.count() << " microseconds" << std::endl;

    return 0;
}
