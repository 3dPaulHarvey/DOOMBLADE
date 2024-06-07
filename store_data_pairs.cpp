#include <iostream>
#include <fstream>
#include <vector>
#include <random>
#include <numeric> // For std::accumulate

int main() {
    // Define the speeds
    std::vector<float> maxSpeeds = {0.015f, 0.025f, 0.035f, 0.045f, 0.055f};
    std::vector<float> averages;  // To store average results for each maxSpeed

    // Setup random number generation
    std::random_device rd;  // Obtain a random number from hardware
    std::mt19937 gen(rd()); // Seed the generator
    std::uniform_real_distribution<> distr(0.0, 1.0); // Define the range

    // Open a file to store the data pairs
    std::ofstream outputFile("speed_data_averages.txt", std::ios::out | std::ios::trunc);
    if (!outputFile.is_open()) {
        std::cerr << "Failed to open the file for writing." << std::endl;
        return 1;
    }

    outputFile << "MaxSpeed, Average Output\n"; // Header

    // Perform tests for each maxSpeed
    for (float speed : maxSpeeds) {
        std::vector<float> results;
        for (int i = 0; i < 4; ++i) { // Four runs per speed setting
            float randomOutput = distr(gen); // Generate a random number
            results.push_back(randomOutput);
        }

        // Calculate the average of the four runs
        float sum = std::accumulate(results.begin(), results.end(), 0.0f);
        float average = sum / results.size();
        averages.push_back(average);

        // Write the average result for this speed to the file
        outputFile << speed << ", " << average << "\n";
    }

    outputFile.close();
    std::cout << "Data has been written to the file successfully." << std::endl;

    return 0;
}
