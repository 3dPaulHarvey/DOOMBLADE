#include "MyController.h"
#include "GraphPlotter.h"
#include "MyGpio.h"
#include "PositionManager.h"

//for storing data
#include <iostream>
#include <fstream>
#include <vector>
#include <numeric> // For std::accumulate

enum class MotorState {
    Initial,
    WaitingToHome,
    Homing,
    Homed,
    WaitingToExtend,
    Extending,
    Extended,
    Sheathing,
    ExtendError,
    SafetyLockout
};

int main() {
    // GPIO SETUP
    MyGpio homeLimitSwitch("gpiochip0", 24);  
    if (!homeLimitSwitch.init()) {
        std::cerr << "Failed to initialize home button" << std::endl;
        return 1;
    }
    MyGpio extendLimitSwitch("gpiochip0", 27); 
    if (!extendLimitSwitch.init()) {
        std::cerr << "Failed to initialize extend button" << std::endl;
        return 1;
    }
    MyGpio activateSwitch("gpiochip0", 17); 
    if (!activateSwitch.init()) {
        std::cerr << "Failed to initialize activate button" << std::endl;
        return 1;
    }
    MyGpio safetySwitch("gpiochip0", 21);  //SAFETY BUTTON

    if (!safetySwitch.init()) {
        std::cerr << "Failed to initialize safety button" << std::endl;
        return 1;
    }

    // CONTROLLER SETUP
    MyController controller("/dev/fdcanusb");
    if (!controller.setupSerialPort()) {
        std::cerr << "Failed to setup serial port" << std::endl;
        return 1;
    }
    controller.sendStopCommand();  //gets controller to a known state
    controller.sendRezeroCommand(500.0f); // sets the current position to 500.0
 
    //Initialization  
    std::vector<float> torques;
    const float startPosition = 500.0f;
    const float cruisingEndPosition = 498.0f; // + 2.2f;
    const float cruisingReverseEndPosition = 501.0f; // - 1.10f;
    const float stepsToAccelerate = 30.0f;  
    const float decelerationSteps = 10.0f; 
    float maxSpeed = 0.055f;       // Max speed in units per control loop iteration
    const float decelerationPerStep = (maxSpeed / stepsToAccelerate);
    std::cout << "Deceleration per step: " << decelerationPerStep << std::endl;
    std::cout << "Deceleration per step x 20 setps: " << decelerationPerStep*20.0f << std::endl;

    struct timespec req = {0, 1200 * 1000}; // Control loop frequency
    float commandedPosition = startPosition;
    float currentPosition = startPosition;
    PositionManager positionManager (controller, homeLimitSwitch, extendLimitSwitch, maxSpeed, cruisingEndPosition, cruisingReverseEndPosition, stepsToAccelerate, decelerationSteps, req);

    controller.sendStopCommand();  //gets controller to a known state
    controller.sendRezeroCommand(500.0f); // sets the current position to 500.0
    commandedPosition = 500.0f;
    float positionAverage = 500.0f;
    bool homingSuccess = positionManager.homing(commandedPosition, currentPosition);
    positionManager.holdPositionDuration(commandedPosition, 1.0f);

    //READY FOR MOTIONS

    // Define the speeds and prepare to store the results
    std::vector<float> maxSpeeds = {0.015f, 0.025f, 0.035f, 0.045f, 0.055f, 0.065f, 0.075f, 0.085f, 0.095f};
    // std::vector<float> maxSpeeds = {0.015f, 0.025f};
    // std::vector<float> maxSpeeds = {0.065f, 0.075f, 0.085f, 0.095f};
    std::vector<float> averages;  // To store average results for each maxSpeed

    // Open a file to store the data pairs
    std::ofstream outputFile("speed_data_pairs.txt", std::ios::out | std::ios::trunc); // Explicitly open in write mode, truncating existing contents
    if (!outputFile.is_open()) {
        std::cerr << "Failed to open the file for writing." << std::endl;
        return 1; // Return with error code
    }


    for (float speed : maxSpeeds) {
        std::vector<float> results;
        //ONE FULL MOTION
        for (int i = 0; i < 2; ++i) { 

            positionManager.changeMaxSpeed(speed);
            controller.sendRezeroCommand(500.0f); // sets the current position to 500.0
            commandedPosition = 500.0f;
            currentPosition = 500.0f;
            positionManager.performAcceleration(commandedPosition, currentPosition);
            positionManager.performCruising(commandedPosition, currentPosition);
            float extendHoldPosition = positionManager.performDeceleration(commandedPosition, currentPosition);
            positionAverage = (commandedPosition + currentPosition) / 2.0f;
            positionManager.holdPositionDuration(positionAverage, 1.0f);

            results.push_back(positionManager.validQuery());
         
            controller.sendRezeroCommand(500.0f); // sets the current position to 500.0
            commandedPosition = 500.0f;
            currentPosition = 500.0f;
            positionManager.holdPositionDuration(commandedPosition, 0.5f);
            controller.sendRezeroCommand(500.0f); // sets the current position to 500.0
            commandedPosition = 500.0f;
            currentPosition = 500.0f;
            positionManager.performAccelerationReverse(commandedPosition, currentPosition);
            positionManager.performCruisingReverse(commandedPosition, currentPosition);
            positionManager.performDecelerationReverse(commandedPosition, currentPosition);
            positionAverage = (commandedPosition + currentPosition) / 2.0f;
            positionManager.holdPositionDuration(positionAverage, 0.1f);
            controller.sendStopCommand();  //gets controller to a known state
            controller.sendRezeroCommand(500.0f); // sets the current position to 500.0
            commandedPosition = 500.0f;
            homingSuccess = positionManager.homing(commandedPosition, currentPosition);
            positionManager.holdPositionDuration(commandedPosition, 1.0f);
        }
        // Calculate the average of the four runs
        float sum = std::accumulate(results.begin(), results.end(), 0.0f);
        float average = sum / results.size();
        averages.push_back(average);

        // Write the average result for this speed to the file
        outputFile << speed << ", " << average << "\n";

    }


    // Close the file
    outputFile.close();
    std::cout << "Data has been written to the file successfully." << std::endl;

    // open a new file and read the data
    std::ifstream inputFile("speed_data_pairs.txt", std::ios::in);
    if (!inputFile.is_open()) {
        std::cerr << "Failed to open the file for reading." << std::endl;
        return 1;
    }

    // Read the data from the file
    std::string line;
    while (std::getline(inputFile, line)) {
        std::cout << line << std::endl;
    }

    // Close the file
    inputFile.close();
    std::cout << "Data has been read from the file successfully." << std::endl;


    return 0;
}
 