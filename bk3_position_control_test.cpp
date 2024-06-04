#include "MyController.h"
#include "MotionManager.h"
#include "GraphPlotter.h"
#include "MyGpio.h"
#include "MyPid.h"




int main() {
    // GPIO SETUP
    MyGpio homeLimitSwitch("gpiochip0", 17);  // Using GPIO 17
    if (!homeLimitSwitch.init()) {
        std::cerr << "Failed to initialize hom button" << std::endl;
        return 1;
    }
    MyGpio extendLimitSwitch("gpiochip0", 27);  // Using GPIO 17
    if (!extendLimitSwitch.init()) {
        std::cerr << "Failed to initialize extend button" << std::endl;
        return 1;
    }

    // CONTROLLER SETUP
    MyController controller("/dev/fdcanusb");
    if (!controller.setupSerialPort()) {
        std::cerr << "Failed to setup serial port" << std::endl;
        return 1;
    }
    controller.sendStopCommand();  //gets controller to a known state
    controller.sendRezeroCommand(500.0); // sets the current position to 500.0
    MotionManager manager(controller, homeLimitSwitch, extendLimitSwitch);
    // float currentPosition = 500.0;
    ////////////////////////////////////////////////////////////////////

    float startingPosition = 0.0f;
    while (1){
        startingPosition =  manager.getInitialPosition();
        if (startingPosition > 400 && startingPosition < 600){ break; }
    }
    manager.holdPositionNanosleep(startingPosition, 0.5);
    std::cout << "Initial Position: " << startingPosition << std::endl;
    ////////////////////////////////////////////////////////////////////


    // Initialization
    std::vector<float> torques;
    const float startPosition = 500.0f;
    const float cruisingEndPosition = 513.5f;
    const size_t stepsToAccelerate = 50;  // Number of steps to reach accelerationEndPosition
    const size_t decelerationSteps = 20; // Number of steps to decelerate, adjust as needed
    const float maxSpeed = 0.055f;       // Max speed in units per control loop iteration
    struct timespec req = {0, 1100 * 1000}; // Control loop frequency: 1100 microseconds

    // Initialize variables for cruising phase
    float stableVelocity = maxSpeed; // Velocity at the end of acceleration
    float currentPosition = startPosition;

    // Deceleration Variables
    float decelerationRate = stableVelocity / decelerationSteps; // Calculate the decrement per step


    // Compute Acceleration phase
    std::vector<float> accelPositions;
    accelPositions.push_back(startPosition);
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
        std::cout << "Accel Position: " << accelPositions[i] << std::endl;
    }

    // Acceleration Phase
    float commandedPosition = accelPositions[0];
    for (size_t i = 0; i < accelPositions.size(); i++) {
        commandedPosition = accelPositions[i];
        controller.sendWriteCommand(commandedPosition, std::numeric_limits<float>::quiet_NaN());
        nanosleep(&req, NULL);
        auto controller_state = controller.sendReadCommand();
        if (controller_state[0] >= 400 && controller_state[0] <= 600) {
            currentPosition = controller_state[0]; // Use valid feedback only
            torques.push_back(controller_state[2]);
        }
        std::cout << "Step: " << i << ", Target: " << accelPositions[i]
                   << ", Actual: " << currentPosition << "\tVelocity" << controller_state[1] << std::endl;


    }

    // Cruising Phase
    while (currentPosition <= cruisingEndPosition) {
        commandedPosition += stableVelocity;
        controller.sendWriteCommand(commandedPosition, std::numeric_limits<float>::quiet_NaN());
        nanosleep(&req, NULL);
        auto controller_state = controller.sendReadCommand();

        if (controller_state[0] >= 400 && controller_state[0] <= 600) {
            currentPosition = controller_state[0]; // Use valid feedback only
            torques.push_back(controller_state[2]);
        }

        std::cout << "Cruising at position: " << currentPosition << "\tVelocity" << controller_state[1] << "\tTorque" << controller_state[2] << std::endl;
    }


    // Deceleration phase 
    currentVelocity = stableVelocity; // Start with the cruising speed
    for (size_t step = 0; step < decelerationSteps; ++step) {
        commandedPosition += currentVelocity; // Apply current velocity to the position
        controller.sendWriteCommand(commandedPosition, std::numeric_limits<float>::quiet_NaN());
        nanosleep(&req, NULL);
        auto controller_state = controller.sendReadCommand();

        currentVelocity -= decelerationRate; // Decrease the current velocity

        if (controller_state[0] >= 350 && controller_state[0] <= 550) {
            currentPosition = controller_state[0];
            torques.push_back(controller_state[2]);
        }
        std::cout << "Decelerating, step " << step + 1 << ": Position = " << controller_state[0] <<  ": Velocity = " << currentVelocity <<std::endl;
        if (currentVelocity <= 0) {
            std::cout << "Deceleration complete at step " << step + 1 << std::endl;
            break; // If velocity is zero or negative, end the deceleration
        }
    }


    manager.holdPositionNanosleep(commandedPosition, 0.5); 

    // Stop the motor
    controller.sendStopCommand();
    controller.closeSerialPort();

    // Graph the torque values collected
    //GraphPlotter plotter;
    //plotter.plot(torques, "Torque Readings Through Various Phases");

    return 0;
}
 