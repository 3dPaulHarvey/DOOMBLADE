#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <cmath> // For std::abs

struct SpeedData {
    float maxSpeed;
    float output;
};

int main() {
    std::string line;
    std::vector<SpeedData> dataPairs;

    // Open the file for reading
    std::ifstream inputFile("speed_data_pairs.txt");
    if (!inputFile.is_open()) {
        std::cerr << "Failed to open the file for reading." << std::endl;
        return 1;
    }

    // Skip the header line
    if (!std::getline(inputFile, line)) {
        std::cerr << "Failed to read the header or the file is empty." << std::endl;
        return 1; // File is empty or header not readable
    }

    // Read data pairs from file
    while (std::getline(inputFile, line)) {
        std::istringstream iss(line);
        SpeedData sd;
        char comma; // to consume the comma between values
        if (!(iss >> sd.maxSpeed >> comma >> sd.output)) {
            std::cerr << "Error reading data from file." << std::endl;
            continue; // Handle reading error or formatting error
        }
        dataPairs.push_back(sd);
    }
    inputFile.close();

    // Check if we have data
    if (dataPairs.empty()) {
        std::cerr << "No data read from file." << std::endl;
        return 1;
    }

    // Modify the last entry's output value
    dataPairs.back().output = 490.524f;

    // Compute differences from the last output
    std::vector<float> differences;
    float lastOutput = dataPairs.back().output;

    for (const auto& sd : dataPairs) {
        float difference = std::abs(sd.output - lastOutput);
        differences.push_back(difference);
    }

    // Optionally save the differences to a new file
    std::ofstream outputFile("output_differences.txt");
    if (!outputFile.is_open()) {
        std::cerr << "Failed to open the file for writing differences." << std::endl;
        return 1;
    }

    outputFile << "MaxSpeed, Difference from Last Output\n";
    for (size_t i = 0; i < dataPairs.size(); i++) {
        outputFile << dataPairs[i].maxSpeed << ", " << differences[i] << "\n";
    }
    outputFile.close();

    std::cout << "Differences have been calculated and saved successfully." << std::endl;

    return 0;
}
