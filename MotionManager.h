#ifndef MOTION_MANAGER_H
#define MOTION_MANAGER_H

#include "MyController.h"
#include "VelocityCurveGenerator.h"
#include <vector>
#include <iostream>
#include <limits>
#include <unistd.h> // For usleep()
#include "MyGpio.h"
#include <cmath>  // Include for std::abs and std::isnormal

///For Nanosleep
#include <time.h>
#include <stdio.h>
#include <errno.h>  //not used


class MotionManager {
public:
    MotionManager(MyController& controller, MyGpio& homeLimitSwitch, MyGpio& extendLimitSwitch)
        : controller(controller), homeLimitSwitch(homeLimitSwitch), extendLimitSwitch(extendLimitSwitch) {}

    void holdPosition(float position, float duration);
    void holdPositionNanosleep(float position, float duration);
    void holdPositionNan(float position, float duration);
    float runAcceleration(float startVel, float endVel, int steps, float goalPosition);
    float runPositionAcceleration(float startVel, float endVel, int steps, float goalPosition);
    float runAccelerationBlind(float startVel, float endVel, int steps);
    float maintainStableVelocity(float velocity, float goalPosition);
    float maintainStableVelocityBlind(float velocity, float goalPosition);
    float getInitialPosition();
    float getPosition();
    float getTorque();
    float homing();
    void homingNanosleep(float initialPosition);

    const std::vector<float>& getTorques() const { return torques; }

private:
    MyController& controller;
    std::vector<float> torques;
    MyGpio& homeLimitSwitch;
    MyGpio& extendLimitSwitch;

    void addTorque(float torque) { torques.push_back(torque); }
};

// HOLD POSITION
void MotionManager::holdPosition(float position, float duration) {
    long holdTime = static_cast<long>(duration * 1000000 / 1025);
    for (long i = 0; i < holdTime; i++) {
        controller.sendWriteCommand(position, 0.0);
        usleep(1100);
        auto controller_state = controller.sendReadCommand();
        addTorque(controller_state[2]);
    }
}

// HOLD POSITION NANO SLEEP
void MotionManager::holdPositionNanosleep(float position, float duration) {
    long holdTime = static_cast<long>(duration * 1000000 / 1025);
    struct timespec req = {0, 1200 * 1000};
    for (long i = 0; i < holdTime; i++) {
        controller.sendWriteCommand(position, 0.0);
        nanosleep(&req, NULL);
        auto controller_state = controller.sendReadCommand();
        addTorque(controller_state[2]);
        //std::cout << "Position: " << controller_state[0] << " Velocity: " << controller_state[1] << " Torque: " << controller_state[2] << std::endl;
    }
}

// HOLD POSITION NAN
void MotionManager::holdPositionNan(float position, float duration) {
    long holdTime = static_cast<long>(duration * 1000000 / 1025);
    for (long i = 0; i < holdTime; i++) {
        controller.sendWriteOnlyCommand(std::numeric_limits<float>::quiet_NaN(), 0.0);
        usleep(1100);
    }
}


//// ACCELERATON SENSED/////////////////////////////////
float  MotionManager::runAcceleration(float startVel, float endVel, int steps, float goalPosition) {
    VelocityCurveGenerator generator(startVel, endVel, steps);
    const auto& velocities = generator.getVelocities();
    float finalVelocity = velocities.back();
    float currentPosition = std::numeric_limits<float>::quiet_NaN(); 
    struct timespec req = {0, 1100 * 1000};
    for (float velocity : velocities) {
        controller.sendWriteCommand(std::numeric_limits<float>::quiet_NaN(), velocity);
        nanosleep(&req, NULL);
        auto controller_state = controller.sendReadCommand();
        currentPosition = controller_state[0];
        //std::cout << "Velocity: " << controller_state[1] << std::endl; 

        // Validate current position
        if (std::abs(currentPosition - goalPosition) <= 15.0f) {
            addTorque(controller_state[2]);
            // Check stopping conditions
            if (controller_state[2] >= 0.5) {  // Check for torque limit indicating a stall or similar issue
                std::cout << "High torque/stall detected, stopping at position: " << currentPosition << std::endl;
                break;
            }
            if (extendLimitSwitch.readValue() == 0) {  // Check if the button is pressed
                std::cout << "Button pressed, stopping at position: " << currentPosition << std::endl;
                break;
            }
        }
    }
    return finalVelocity;
}


//// ACCELERATON POSITION/////////////////////////////////
float  MotionManager::runPositionAcceleration(float startVel, float endVel, int steps, float goalPosition) {

    return 1.0;
}


//// STABLE VELOCITY /////////////////////////////////////
float MotionManager::maintainStableVelocity(float velocity, float goalPosition) {
    std::cout << "Maintaining stable velocity..." << std::endl;
    int index = 0;  // Index for inserting positions into the array
    float currentPosition = std::numeric_limits<float>::quiet_NaN(); // Initialize with NaN

    do {
        controller.sendWriteCommand(std::numeric_limits<float>::quiet_NaN(), velocity);
        usleep(1250);
        auto controller_state = controller.sendReadCommand();
        currentPosition = controller_state[0];

        // Validate current position
        if (std::abs(currentPosition - goalPosition) <= 15.0f) {
            addTorque(controller_state[2]);

            // Check stopping conditions
            if (controller_state[2] >= 0.8) {  // Check for torque limit indicating a stall or similar issue
                std::cout << "High torque/stall detected, stopping at position: " << currentPosition << std::endl;
                break;
            }
            if (currentPosition <= goalPosition) {
                std::cout << "Reached goal position: " << currentPosition << std::endl;
                break;
            }
            if (extendLimitSwitch.readValue() == 0) {  // Check if the button is pressed
                std::cout << "Button pressed, stopping at position: " << currentPosition << std::endl;
                break;
            }
        }
    } while (true);

    return currentPosition;  // Return the last valid position
}
// STABLE VELOCITY BLIND
float MotionManager::maintainStableVelocityBlind(float velocity, float goalPosition) {
    std::cout << "Maintaining stable velocity..." << std::endl;

    do {
        controller.sendWriteOnlyCommand(std::numeric_limits<float>::quiet_NaN(), velocity);
        usleep(5);
        if (extendLimitSwitch.readValue() == 0) {  // Check if the button is pressed
            std::cout << "Button pressed, stopping at position: " << std::endl;
            break;
        } 
    } while (true);

    return 0.0;  // Return the last valid position
}


// QUERY INITIAL
float MotionManager::getInitialPosition() {
    std::cout << "Querying initial position..." << std::endl;
    float initialPosition = 0.0f;
    struct timespec req = {0, 900 * 1000};
    for (int i = 0; i < 3; i++) {
        controller.sendQueryCommand();
        nanosleep(&req, NULL);
        auto controller_state = controller.sendReadCommand();
        std::cout << "Query " << i << " Position: " << controller_state[0] << std::endl;  //remove
        if (i == 2) {  // Use only the last query result
            initialPosition = controller_state[0];  // Assuming position is at index 0
        }
    }
    std::cout << "Confirmed Initial Position: " << initialPosition << std::endl;   //remove
    return initialPosition;
}
// GET POSITION
float MotionManager::getPosition() {
    std::cout << "Querying initial position..." << std::endl;
    float initialPosition = 0.0f;
    struct timespec req = {0, 900 * 1000};
        controller.sendQueryCommand();
        nanosleep(&req, NULL);
        auto controller_state = controller.sendReadCommand();
        initialPosition = controller_state[0];    
    std::cout << "Confirmed Initial Position: " << initialPosition << std::endl;
    return initialPosition;
}

//GET TORQUE
float MotionManager::getTorque() {
    std::cout << "Querying initial position..." << std::endl;
    float initialPosition = 0.0f;
    struct timespec req = {0, 800 * 1000};
        controller.sendQueryCommand();
        nanosleep(&req, NULL);
        auto controller_state = controller.sendReadCommand();
        initialPosition = controller_state[2];    
    std::cout << "Confirmed Initial Position: " << initialPosition << std::endl;
    return initialPosition;
}

// HOMING NANOSLEEP
void MotionManager::homingNanosleep(float initialPosition) {    
    std::cout << "Starting Homing Blind..." << std::endl;
    float currentPosition = std::numeric_limits<float>::quiet_NaN(); // Initialize with NaN
    int index = 0;
    struct timespec req = {0, 1200 * 1000};
    
    while (true) {
        controller.sendWriteCommand(std::numeric_limits<float>::quiet_NaN(), 4.5);
        nanosleep(&req, NULL);
        auto controller_state = controller.sendReadCommand();
        index++;
        currentPosition = controller_state[0];  // Assuming position is at index 0
        if (std::abs(currentPosition - initialPosition) <= 15.0) {
            //print out the torque
            addTorque(controller_state[2]);
            // Check for obstruction
            if (index > 13) {
                if (controller_state[2] >= 0.05) {  // Check for torque limit indicating a stall or similar issue
                    std::cout << "High torque/stall detected, stopping at position: " << currentPosition << "at index" << index << std::endl;
                        //Move forward a little bit
                    for (int i = 0; i < 100; i++) {
                        controller.sendWriteCommand(std::numeric_limits<float>::quiet_NaN(), -4.5);
                        nanosleep(&req, NULL);
                        auto controller_state = controller.sendReadCommand();
                        addTorque(controller_state[2]);
                    }
                    break;
                }
            }
            // Check if the button is pressed
            if (homeLimitSwitch.readValue() == 0) {  
                std::cout << "Button pressed, stopping at position: " << currentPosition << std::endl;
                break;
            }
        }
    }
    return; 
} // End HOMING



#endif // MOTION_MANAGER_H
