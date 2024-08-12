#ifndef POSITION_MANAGER_H
#define POSITION_MANAGER_H

#include "MyController.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <ctime>

#define DEBUG 0

class PositionManager {
public:
    // Constructor
    explicit PositionManager(MyController& controller, MyGpio& homeLimitSwitch, MyGpio& extendLimitSwitch, float maxSpeed, float cruisingEndPosition, float cruisingReverseEndPosition, size_t stepsToAccelerate, size_t decelerationSteps, struct timespec req)
    : controller(controller), homeLimitSwitch(homeLimitSwitch), extendLimitSwitch(extendLimitSwitch), maxSpeed(maxSpeed), cruisingEndPosition(cruisingEndPosition), cruisingReverseEndPosition(cruisingReverseEndPosition), 
      stepsToAccelerate(stepsToAccelerate), decelerationSteps(decelerationSteps), req(req) {}

    
    // Motor control functions
    inline void performAcceleration(float& commandedPosition, float& currentPosition);
    inline void performCruising(float& commandedPosition, float& currentPosition);
    inline float performDeceleration(float& commandedPosition, float& currentPosition);
    inline void catchCruising(float& commandedPosition, float& currentPosition);

    inline void performAccelerationReverse(float& commandedPosition, float& currentPosition);
    inline void performCruisingReverse(float& commandedPosition, float& currentPosition);
    inline void performDecelerationReverse(float& commandedPosition, float& currentPosition);

    
    inline void holdPosition(float position);
    inline void holdPositionDuration(float position, float duration);
    inline void holdPositionNan();
    inline void holdPositionNanDuration(float duration);
    inline bool homing(float& commandedPosition, float& currentPosition);
    inline float tripleQuery();
    inline float validQuery();
    inline void changeMaxSpeed(float newMaxSpeed);
    inline double calculateDecelerationDistance(double initialVelocity, size_t numSteps, double timePerStep);


    // Accessor for torque data
    inline const std::vector<float>& getTorques() const { return torques; }

private:
    MyController& controller;
    float maxSpeed;
    float cruisingEndPosition;
    float cruisingReverseEndPosition;
    size_t stepsToAccelerate;
    size_t decelerationSteps;
    struct timespec req;
    std::vector<float> torques;
    MyGpio& homeLimitSwitch;
    MyGpio& extendLimitSwitch;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////


//function to change maxSpeed
void PositionManager::changeMaxSpeed(float newMaxSpeed) {
    maxSpeed = newMaxSpeed;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////


// ACCELERATION
void PositionManager::performAcceleration(float& commandedPosition, float& currentPosition) {
    std::cout << "ACCELERATION" << std::endl;
    std::vector<float> accelPositions;
    accelPositions.push_back(currentPosition); // Assuming currentPosition is rezeroed at 500.0

    float currentVelocity = 0.0f;
    float accelerationPerStep = -maxSpeed / stepsToAccelerate; 
    std::cout << "Acceleration Per Step: " << accelerationPerStep << std::endl;

    for (int i = 1; i <= stepsToAccelerate; i++) {
        currentVelocity += accelerationPerStep;
        float newPosition = accelPositions.back() + currentVelocity;
        accelPositions.push_back(newPosition);
        std::cout << "CurrentVelocity: " << currentVelocity << "\tNewPosition: " << newPosition << std::endl;
    }

    for (size_t i = 0; i < accelPositions.size(); i++) {
        commandedPosition = accelPositions[i];
        controller.sendWriteCommand(commandedPosition, std::numeric_limits<float>::quiet_NaN());
        nanosleep(&req, nullptr);
        auto controller_state = controller.sendReadCommand();
        if (controller_state[0] >= 450 && controller_state[0] <= 550) {
            currentPosition = controller_state[0];
            //torques.push_back(controller_state[2]);
        }
        std::cout << i << "\t" << controller_state[2] << "\tTarget: " << commandedPosition
                  << "\tActual: " << currentPosition << "\t" << std::abs(commandedPosition - currentPosition) <<  "\tVelocity: " << controller_state[1] << std::endl;



    }
}

// CRUISING 
void PositionManager::performCruising(float& commandedPosition, float& currentPosition) {
    std::cout << "CRUISING" << std::endl;
    float stableVelocity = -maxSpeed;
    int index = 0;
    while (currentPosition >= cruisingEndPosition) {
        commandedPosition += stableVelocity;
        controller.sendWriteCommand(commandedPosition, std::numeric_limits<float>::quiet_NaN());
        nanosleep(&req, nullptr);
        auto controller_state = controller.sendReadCommand();
        index++;
        if (controller_state[0] >= 450 && controller_state[0] <= 550) {
            currentPosition = controller_state[0];
            // torques.push_back(controller_state[2]);
            // std::cout << "torque: " << controller_state[2] << std::endl;

            if (index > 23) {
                if (controller_state[2] >= 0.20) {  // Check for torque limit indicating a stall or similar issue
                    std::cout << "High torque/stall detected, stopping at position: " << currentPosition << "at index" << index << std::endl;
                    //Move forward a little bit
                    std::cout << "extendLimitSwitch.readValue() " << extendLimitSwitch.readValue() << std::endl;

                    // long holdTime = static_cast<long>(0.05 * 1000000 / 1025);
                    // struct timespec req = {0, 1200 * 1000};
                    // for (long i = 0; i < holdTime; i++) {
                    // controller.sendWriteCommand(std::numeric_limits<float>::quiet_NaN(), 0.0);
                    // nanosleep(&req, NULL);
                    // auto controller_state = controller.sendReadCommand();
                    // if (controller_state[0] >= 450 && controller_state[0] <= 550) {
                    //     torques.push_back(controller_state[2]);
                    // }

                    // for (int i = 0; i < 250; i++) {
                    //     //ramp up to -2.5
                    //     nanosleep(&req, NULL);
                    //     torques.push_back(controller_state[2]);
                    // }
                    controller.sendStopCommand();  //gets controller to a known state
                    controller.sendRezeroCommand(500.0f); // sets the current position to 500.0f
                    commandedPosition = 500.0f;
                    currentPosition = 500.0f;
                    

                    // for (int i = 0; i < 100; i++) {
                    //     //ramp up to -2.5
                    //     controller.sendWriteCommand(std::numeric_limits<float>::quiet_NaN(), -2.5 * (i / 100.0));
                    //     nanosleep(&req, NULL);        
                    //     auto controller_state = controller.sendReadCommand();
                    //     torques.push_back(controller_state[2]);
                    // }
                    for (int i = 0; i < 250; i++) {
                        //ramp up to -2.5
                        controller.sendWriteCommand(std::numeric_limits<float>::quiet_NaN(), -2.5);
                        nanosleep(&req, NULL);        
                        auto controller_state = controller.sendReadCommand();
                        torques.push_back(controller_state[2]);
                    }
                    return;                    
                }
            }
        }
        std::cout << index << "\t" << controller_state[2] << "\tTarget: " << commandedPosition
                  << "\tActual: " << currentPosition << "\t" << std::abs(commandedPosition - currentPosition) <<  "\tVelocity: " << controller_state[1] << std::endl;


    }// end while
}

// DECELERATION
float PositionManager::performDeceleration(float& commandedPosition, float& currentPosition) {
    float stableReverseVelocity = -maxSpeed;
    float currentVelocity = stableReverseVelocity;
    float decelerationRate = -stableReverseVelocity / decelerationSteps;
    int index = 0; 
    std::cout << "DECELERATION" << std::endl;

    for (size_t step = 0; step < decelerationSteps; ++step) {
        index++;
        float startPositionThisStep = currentPosition;
        
        commandedPosition += currentVelocity;
        controller.sendWriteCommand(commandedPosition, std::numeric_limits<float>::quiet_NaN());
        nanosleep(&req, nullptr);
        
        auto controller_state = controller.sendReadCommand();
        if (controller_state[0] >= 450 && controller_state[0] <= 550) {
            currentPosition = controller_state[0];
            //torques.push_back(controller_state[0]);
        }
        
        currentVelocity -= decelerationRate;
        
        std::cout << index << "\t" << controller_state[2] << "\tTarget: " << commandedPosition
                  << "\tActual: " << currentPosition << "\t" << std::abs(commandedPosition - currentPosition) <<  "\tVelocity: " << controller_state[1] << std::endl;

        //std::cout << "Commanded Position = " << commandedPosition << ", Actual Position = " << currentPosition << "\tVelocity" << controller_state[1] << "\tTorque" << controller_state[2] << std::endl;

        if (currentVelocity >= 0) { // Check for stopping condition
            std::cout << "Deceleration complete at step " << step + 1 << std::endl;
            break;
        }
        if (extendLimitSwitch.readValue() == 0) {  
            std::cout << "Button pressed, stopping at position: " << currentPosition << std::endl;
            break;
        }
    }
    return currentPosition;
}



///////////////////////////////////////////////////////////////////////////////////////////////////////
// ACCELERATION REVERSE
void PositionManager::performAccelerationReverse(float& commandedPosition, float& currentPosition) {
    std::cout << "ACCELERATION REVERSE" << std::endl;
    std::vector<float> accelPositions;
    accelPositions.push_back(currentPosition);

    float currentVelocity = 0.0f;
    float accelerationPerStep = maxSpeed / stepsToAccelerate;
    std::cout << "Acceleration Per Step: " << accelerationPerStep << std::endl;

    for (int i = 1; i <= stepsToAccelerate; i++) {
        currentVelocity += accelerationPerStep;
        float newPosition = accelPositions.back() + currentVelocity;
        accelPositions.push_back(newPosition);
        std::cout << "Current Velocity: " << currentVelocity << "\tNew Position: " << newPosition << std::endl;
    }

    for (size_t i = 0; i < accelPositions.size(); i++) {
        commandedPosition = accelPositions[i];
        controller.sendWriteCommand(commandedPosition, std::numeric_limits<float>::quiet_NaN());
        nanosleep(&req, nullptr);
        auto controller_state = controller.sendReadCommand();
        if (controller_state[0] >= 450 && controller_state[0] <= 550) {
            currentPosition = controller_state[0];
            torques.push_back(controller_state[2]);
        }
        std::cout << i << "\t" << controller_state[2] << "\tTarget: " << commandedPosition
                  << "\tActual: " << currentPosition << "\t" << std::abs(commandedPosition - currentPosition) <<  "\tVelocity: " << controller_state[1] << std::endl;

    }
}

// CRUISING REVERSE
void PositionManager::performCruisingReverse(float& commandedPosition, float& currentPosition) {
    std::cout << "CRUISING REVERSE" << std::endl;
    torques.push_back(0.0);
    float stableVelocity = maxSpeed;
    int index = 0;
    while (currentPosition <= cruisingReverseEndPosition) {
        commandedPosition += stableVelocity;
        controller.sendWriteCommand(commandedPosition, std::numeric_limits<float>::quiet_NaN());
        nanosleep(&req, nullptr);
        auto controller_state = controller.sendReadCommand();
        index++;
        if (controller_state[0] >= 450 && controller_state[0] <= 550) {
            currentPosition = controller_state[0];
            torques.push_back(controller_state[2]);
            // std::cout << "torque: " << controller_state[2] << std::endl;

            if (index >7) {  //13
                if (controller_state[2] >= 0.40) {  // Check for torque limit indicating a stall or similar issue
                    std::cout << "High torque/stall detected, stopping at position: " << currentPosition << "at index" << index << std::endl;
                    //Move forward a little bit
                    std::cout << "extendLimitSwitch.readValue() " << extendLimitSwitch.readValue() << std::endl;

                    // long holdTime = static_cast<long>(0.05 * 1000000 / 1025);
                    // struct timespec req = {0, 1200 * 1000};
                    // for (long i = 0; i < holdTime; i++) {
                    // controller.sendWriteCommand(std::numeric_limits<float>::quiet_NaN(), 0.0);
                    // nanosleep(&req, NULL);
                    // auto controller_state = controller.sendReadCommand();
                    // if (controller_state[0] >= 450 && controller_state[0] <= 550) {
                    //     torques.push_back(controller_state[2]);
                    // }

                    // for (int i = 0; i < 250; i++) {
                    //     //ramp up to -2.5
                    //     nanosleep(&req, NULL);
                    //     torques.push_back(controller_state[2]);
                    // }
                    controller.sendStopCommand();  //gets controller to a known state
                    controller.sendRezeroCommand(500.0f); // sets the current position to 500.0f
                    commandedPosition = 500.0f;
                    currentPosition = 500.0f;
                    

                    // for (int i = 0; i < 100; i++) {
                    //     //ramp up to -2.5
                    //     controller.sendWriteCommand(std::numeric_limits<float>::quiet_NaN(), -2.5 * (i / 100.0));
                    //     nanosleep(&req, NULL);        
                    //     auto controller_state = controller.sendReadCommand();
                    //     torques.push_back(controller_state[2]);
                    // }
                    for (int i = 0; i < 250; i++) {
                        //ramp up to -2.5
                        controller.sendWriteCommand(std::numeric_limits<float>::quiet_NaN(), -2.5);
                        nanosleep(&req, NULL);        
                        auto controller_state = controller.sendReadCommand();
                        torques.push_back(controller_state[2]);
                    }
                    return;                    
                }
            }
        }
        std::cout << index << "\t" << controller_state[2] << "\tTarget: " << commandedPosition
                  << "\tActual: " << currentPosition << "\t" << std::abs(commandedPosition - currentPosition) <<  "\tVelocity: " << controller_state[1] << std::endl;


    }
}

// DECELERATION REVERSE
void PositionManager::performDecelerationReverse(float& commandedPosition, float& currentPosition) {
    std::cout << "DECELERATION REVERSE" << std::endl;
    float stableVelocity = maxSpeed;
    float currentVelocity = stableVelocity;
    float decelerationRate = stableVelocity / decelerationSteps;
    int index = 0;

    for (size_t step = 0; step < decelerationSteps; ++step) {
        index++;
        commandedPosition += currentVelocity;
        controller.sendWriteCommand(commandedPosition, std::numeric_limits<float>::quiet_NaN());
        nanosleep(&req, nullptr);
        auto controller_state = controller.sendReadCommand();
        currentVelocity -= decelerationRate;
        if (controller_state[0] >= 450 && controller_state[0] <= 550) {
            currentPosition = controller_state[0];
            torques.push_back(controller_state[2]);
        }
        std::cout << index << "\t" << controller_state[2] << "\tTarget: " << commandedPosition
                  << "\tActual: " << currentPosition << "\t" << std::abs(commandedPosition - currentPosition) <<  "\tVelocity: " << controller_state[1] << std::endl;;

        if (currentVelocity <= 0) {
            std::cout << "Deceleration complete at step " << step + 1 << std::endl;
            break;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// HOLD POSITION 
void PositionManager::holdPosition(float position) {

    struct timespec req = {0, 1200 * 1000};
    controller.sendWriteCommand(position, 0.0);
    nanosleep(&req, NULL);
    auto controller_state = controller.sendReadCommand();
    if (controller_state[0] >= 450 && controller_state[0] <= 550) {
        //torques.push_back(controller_state[2]);
    }
    //std::cout << "Position: " << controller_state[0] << " Velocity: " << controller_state[1] << " Torque: " << controller_state[2] << std::endl;
}

// HOLD POSITION DURATION
void PositionManager::holdPositionDuration(float position, float duration) {
    long holdTime = static_cast<long>(duration * 1000000 / 1025);
    struct timespec req = {0, 1200 * 1000};
    for (long i = 0; i < holdTime; i++) {
        controller.sendWriteCommand(position, 0.0);
        nanosleep(&req, NULL);
        auto controller_state = controller.sendReadCommand();
        if (controller_state[0] >= 450 && controller_state[0] <= 550) {
            //torques.push_back(controller_state[0]);
        }
        //std::cout << "Position: " << controller_state[0] << " Velocity: " << controller_state[1] << " Torque: " << controller_state[2] << std::endl;
    }
}

// HOLD POSITION NAN
void PositionManager::holdPositionNan() {

    struct timespec req = {0, 1200 * 1000};
    controller.sendWriteCommand(std::numeric_limits<float>::quiet_NaN(), 0.0);
    nanosleep(&req, NULL);
    auto controller_state = controller.sendReadCommand();
    if (controller_state[0] >= 450 && controller_state[0] <= 550) {
        //torques.push_back(controller_state[2]);
    }
    //std::cout << "Position: " << controller_state[0] << " Velocity: " << controller_state[1] << " Torque: " << controller_state[2] << std::endl;
}

// HOLD POSITION NAN Duration
void PositionManager::holdPositionNanDuration(float duration) {
    long holdTime = static_cast<long>(duration * 1000000 / 1025);
    struct timespec req = {0, 1200 * 1000};
    controller.sendWriteCommand(std::numeric_limits<float>::quiet_NaN(), 0.0);
    nanosleep(&req, NULL);
    auto controller_state = controller.sendReadCommand();
    if (controller_state[0] >= 450 && controller_state[0] <= 550) {
        //torques.push_back(controller_state[2]);
    }
    //std::cout << "Position: " << controller_state[0] << " Velocity: " << controller_state[1] << " Torque: " << controller_state[2] << std::endl;
}


// HOMING
bool PositionManager::homing(float& commandedPosition, float& currentPosition) {    
    std::cout << "Starting Homing..." << std::endl;
    controller.sendRezeroCommand(500.0); // sets the current position to 500.0
    int index = 0;
    struct timespec req = {0, 1200 * 1000};
    
    while (true) {
        if (homeLimitSwitch.readValue() == 0) {  
            std::cout << "Button pressed, stopping at position: " << commandedPosition << std::endl;
            std::cout << "Button pressed, stopping at position: " << currentPosition << std::endl;
            break;
        }
        controller.sendWriteCommand(std::numeric_limits<float>::quiet_NaN(), 4.5);
        nanosleep(&req, NULL);
        auto controller_state = controller.sendReadCommand();
        index++;
        if (controller_state[0] >= 450 && controller_state[0] <= 550) {
            currentPosition = controller_state[0];
            commandedPosition = currentPosition;
            //std::cout << "Current Position: " << controller_state[0] << " Velocity: " << controller_state[1] << " Torque: " << controller_state[2] << std::endl;
            //torques.push_back(controller_state[2]);
            if (index > 20) {
                if (controller_state[2] >= 0.13) {  // Check for torque limit indicating a stall or similar issue
                    std::cout << "High torque/stall detected, stopping at position: " << currentPosition << "at index" << index << std::endl;
                    //Move forward a little bit
                    std::cout << "extendLimitSwitch.readValue() " << extendLimitSwitch.readValue() << std::endl;
                    if (!extendLimitSwitch.readValue() == 0){
                        for (int i = 0; i < 100; i++) {
                            controller.sendWriteCommand(std::numeric_limits<float>::quiet_NaN(), -4.5);
                            nanosleep(&req, NULL);
                            auto controller_state = controller.sendReadCommand();
                            //torques.push_back(controller_state[2]);
                        }
                    }
                    controller.sendRezeroCommand(500.0f); // sets the current position to 500.0
                    commandedPosition = 500.0f;
                    currentPosition = 500.0f;
                    return false;  //obstruction encountered
                }
            }
            // Check if the button is pressed
        }
    }
    //Clean Up
    controller.sendRezeroCommand(500.0); // sets the current position to 500.0
    commandedPosition = 500.0f;
    currentPosition = 500.0f;
    std::cout << "HOMED" << std::endl; 
    return true; 
} // End HOMING


// TRIPLE QUERY
float PositionManager::tripleQuery() {
    std::cout << "Querying initial position..." << std::endl;
    struct timespec req = {0, 900 * 1000};
    std::vector<float> positions(3);

    while (true) {
        // Collect the three position queries
        for (int i = 0; i < 3; i++) {
            controller.sendQueryCommand();
            nanosleep(&req, NULL);
            auto controller_state = controller.sendReadCommand();
            positions[i] = controller_state[0];  // Store each position in the vector
            std::cout << "Query " << i << " Position: " << positions[i] << std::endl;
        }

        // Check if any position is within the range of 450 to 550
        for (float position : positions) {
            if (position >= 450.0f && position <= 550.0f) {
                std::cout << "Confirmed Initial Position within range: " << position << std::endl;
                return position;  // Return the first position within the range
            }
        }

        // If no position is in the range, print a message and the loop will run again
        std::cout << "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX" << std::endl; 
        std::cout << "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX" << std::endl; 
        std::cout << "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX" << std::endl; 
        std::cout << "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX" << std::endl; 
    }
}


//VALID QUERY
float PositionManager::validQuery() {
    std::cout << "Querying position..." << std::endl;
    struct timespec req = {0, 900 * 1000};
    int attemptCount = 0;
    const int maxAttempts = 5;
    const float lowerBound = 450.0f;
    const float upperBound = 550.0f;

    while (attemptCount < maxAttempts) {
        controller.sendQueryCommand();
        nanosleep(&req, NULL);
        auto controller_state = controller.sendReadCommand();
        float position = controller_state[0];

        std::cout << "Attempt " << attemptCount + 1 << ": Position = " << position << std::endl;

        if (position >= lowerBound && position <= upperBound) {
            std::cout << "Valid position found within range: " << position << std::endl;
            return position;
        }

        attemptCount++;
    }

    std::cout << "Failed to find a valid position after " << maxAttempts << " attempts." << std::endl;
    return -1;  // Indicate failure to find a valid position
}




#endif // POSITION_MANAGER_H
