#ifndef POSITION_MANAGER_H
#define POSITION_MANAGER_H

#include "MyController.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <ctime>

class PositionManager {
public:
    // Constructor
    explicit PositionManager(MyController& controller, MyGpio& homeLimitSwitch, MyGpio& extendLimitSwitch, float maxSpeed, float cruisingEndPosition, float cruisingReverseEndPosition, size_t stepsToAccelerate, size_t decelerationSteps, struct timespec req)
    : controller(controller), homeLimitSwitch(homeLimitSwitch), extendLimitSwitch(extendLimitSwitch), maxSpeed(maxSpeed), cruisingEndPosition(cruisingEndPosition), cruisingReverseEndPosition(cruisingReverseEndPosition), 
      stepsToAccelerate(stepsToAccelerate), decelerationSteps(decelerationSteps), req(req) {}

    // Motor control functions
    inline void holdPosition(float position);
    inline void holdPositionDuration(float position, float duration);
    inline bool homing(float& commandedPosition, float& currentPosition);
    
    inline void performAcceleration(float& commandedPosition, float& currentPosition);
    inline void performCruising(float& commandedPosition, float& currentPosition);
    inline void performDeceleration(float& commandedPosition, float& currentPosition);

    inline void performAccelerationReverse(float& commandedPosition, float& currentPosition);
    inline void performCruisingReverse(float& commandedPosition, float& currentPosition);
    inline void performDecelerationReverse(float& commandedPosition, float& currentPosition);
    

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


// ACCELERATION
void PositionManager::performAcceleration(float& commandedPosition, float& currentPosition) {
    std::vector<float> accelPositions;
    accelPositions.push_back(currentPosition); // Assuming currentPosition is rezeroed at 500.0

    float currentVelocity = 0.0f;
    float accelerationPerStep = -maxSpeed / stepsToAccelerate; // Negative for reverse direction
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
            torques.push_back(controller_state[2]);
        }
        std::cout << "Step: " << i << ", Target: " << accelPositions[i]
                  << ", Actual: " << currentPosition << "\tVelocity" << controller_state[1] << std::endl;
    }
}

// CRUISING 
void PositionManager::performCruising(float& commandedPosition, float& currentPosition) {
    float stableReverseVelocity = -maxSpeed; // Negative for reverse direction
    while (currentPosition >= cruisingEndPosition) { // Assumes cruisingEndPosition is adjusted for reverse
        commandedPosition += stableReverseVelocity;
        controller.sendWriteCommand(commandedPosition, std::numeric_limits<float>::quiet_NaN());
        nanosleep(&req, nullptr);
        auto controller_state = controller.sendReadCommand();
        if (controller_state[0] >= 450 && controller_state[0] <= 550) {
            currentPosition = controller_state[0];
            torques.push_back(controller_state[2]);
        }
        std::cout << "Cruising at position: " << currentPosition << "\tVelocity" << controller_state[1] << "\tTorque" << controller_state[2] << std::endl;
    }
}

// DECELERATION
void PositionManager::performDeceleration(float& commandedPosition, float& currentPosition) {
    float stableReverseVelocity = -maxSpeed; // Start with the cruising reverse speed
    float currentVelocity = stableReverseVelocity;
    float decelerationRate = -stableReverseVelocity / decelerationSteps; // Negative for reverse direction

    for (size_t step = 0; step < decelerationSteps; ++step) {
        commandedPosition += currentVelocity;
        controller.sendWriteCommand(commandedPosition, std::numeric_limits<float>::quiet_NaN());
        nanosleep(&req, nullptr);
        auto controller_state = controller.sendReadCommand();
        currentVelocity -= decelerationRate; // Decrease the current reverse velocity
        if (controller_state[0] >= 450 && controller_state[0] <= 550) {
            currentPosition = controller_state[0];
            torques.push_back(controller_state[2]);
        }
        std::cout << "Decelerating, step " << step + 1 << ": Position = " << controller_state[0]
                  << ": Velocity = " << controller_state[1] << std::endl;
        if (currentVelocity >= 0) { // Check for stopping condition
            std::cout << "Deceleration complete at step " << step + 1 << std::endl;
            break;
        }
    }
}




///////////////////////////////////////////////////////////////////////////////////////////////////////
// ACCELERATION REVERSE
void PositionManager::performAccelerationReverse(float& commandedPosition, float& currentPosition) {
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
        std::cout << "Step: " << i << ", Target: " << accelPositions[i]
                  << ", Actual: " << currentPosition << "\tVelocity" << controller_state[1] << std::endl;
    }
}

// CRUISING REVERSE
void PositionManager::performCruisingReverse(float& commandedPosition, float& currentPosition) {
    float stableVelocity = maxSpeed;
    while (currentPosition <= cruisingReverseEndPosition) {
        commandedPosition += stableVelocity;
        controller.sendWriteCommand(commandedPosition, std::numeric_limits<float>::quiet_NaN());
        nanosleep(&req, nullptr);
        auto controller_state = controller.sendReadCommand();
        if (controller_state[0] >= 450 && controller_state[0] <= 550) {
            currentPosition = controller_state[0];
            torques.push_back(controller_state[2]);
        }
        std::cout << "Cruising at position: " << currentPosition << "\tVelocity" << controller_state[1] << "\tTorque" << controller_state[2] << std::endl;
    }
}

// DECELERATION REVERSE
void PositionManager::performDecelerationReverse(float& commandedPosition, float& currentPosition) {
    float stableVelocity = maxSpeed;
    float currentVelocity = stableVelocity;
    float decelerationRate = stableVelocity / decelerationSteps;

    for (size_t step = 0; step < decelerationSteps; ++step) {
        commandedPosition += currentVelocity;
        controller.sendWriteCommand(commandedPosition, std::numeric_limits<float>::quiet_NaN());
        nanosleep(&req, nullptr);
        auto controller_state = controller.sendReadCommand();
        currentVelocity -= decelerationRate;
        if (controller_state[0] >= 450 && controller_state[0] <= 550) {
            currentPosition = controller_state[0];
            torques.push_back(controller_state[2]);
        }
        std::cout << "Decelerating, step " << step + 1 << ": Position = " << controller_state[0]
                  << ": Velocity = " << controller_state[1] << std::endl;
        if (currentVelocity <= 0) {
            std::cout << "Deceleration complete at step " << step + 1 << std::endl;
            break;
        }
    }
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
            torques.push_back(controller_state[2]);
        }
        //std::cout << "Position: " << controller_state[0] << " Velocity: " << controller_state[1] << " Torque: " << controller_state[2] << std::endl;
    }
}

// HOLD POSITION 
void PositionManager::holdPosition(float position) {

    struct timespec req = {0, 1200 * 1000};
    controller.sendWriteCommand(position, 0.0);
    nanosleep(&req, NULL);
    auto controller_state = controller.sendReadCommand();
    if (controller_state[0] >= 450 && controller_state[0] <= 550) {
        torques.push_back(controller_state[2]);
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
        controller.sendWriteCommand(std::numeric_limits<float>::quiet_NaN(), 4.5);
        nanosleep(&req, NULL);
        auto controller_state = controller.sendReadCommand();
        index++;
        if (controller_state[0] >= 450 && controller_state[0] <= 550) {
            currentPosition = controller_state[0];
            //std::cout << "Current Position: " << controller_state[0] << " Velocity: " << controller_state[1] << " Torque: " << controller_state[2] << std::endl;
            //torques.push_back(controller_state[2]);
            if (index > 13) {
                if (controller_state[2] >= 0.06) {  // Check for torque limit indicating a stall or similar issue
                    std::cout << "High torque/stall detected, stopping at position: " << currentPosition << "at index" << index << std::endl;
                        //Move forward a little bit
                    for (int i = 0; i < 100; i++) {
                        controller.sendWriteCommand(std::numeric_limits<float>::quiet_NaN(), -4.5);
                        nanosleep(&req, NULL);
                        auto controller_state = controller.sendReadCommand();
                        torques.push_back(controller_state[2]);
                    }
                    return false;
                }
            }
            // Check if the button is pressed
            if (homeLimitSwitch.readValue() == 0) {  
                std::cout << "Button pressed, stopping at position: " << currentPosition << std::endl;
                break;
            }
        }
    }
    //Clean Up
    controller.sendRezeroCommand(500.0); // sets the current position to 500.0
    commandedPosition = 500.0f;
    currentPosition = 500.0f;
    std::cout << "HOMED" << std::endl; 
    return true; 
} // End HOMING

#endif // POSITION_MANAGER_H
