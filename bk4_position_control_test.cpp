#include "MyController.h"
#include "GraphPlotter.h"
#include "MyGpio.h"
#include "PositionManager.h"



int main() {
    // GPIO SETUP
    MyGpio homeLimitSwitch("gpiochip0", 24);  
    if (!homeLimitSwitch.init()) {
        std::cerr << "Failed to initialize hom button" << std::endl;
        return 1;
    }
    MyGpio extendLimitSwitch("gpiochip0", 27); 
    if (!extendLimitSwitch.init()) {
        std::cerr << "Failed to initialize extend button" << std::endl;
        return 1;
    }
    MyGpio activateSwitch("gpiochip0", 17); 
    if (!activateSwitch.init()) {
        std::cerr << "Failed to initialize extend button" << std::endl;
        return 1;
    }
    MyGpio safetySwitch("gpiochip0", 26); 
    if (!activateSwitch.init()) {
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
    controller.sendRezeroCommand(500.0f); // sets the current position to 500.0


    // Initialization
    std::vector<float> torques;
    const float startPosition = 500.0f;
    const float cruisingEndPosition = 480.0f; // + 2.2f;
    const float cruisingReverseEndPosition = 520.0f; // - 1.10f;
    const float stepsToAccelerate = 50.0f;  
    const float decelerationSteps = 20.0f; 
    const float maxSpeed = 0.055f;       // Max speed in units per control loop iteration
    struct timespec req = {0, 1200 * 1000}; // Control loop frequency
    float commandedPosition = startPosition;
    float currentPosition = startPosition;
    PositionManager positionManager (controller, homeLimitSwitch, extendLimitSwitch, maxSpeed, cruisingEndPosition, cruisingReverseEndPosition, stepsToAccelerate, decelerationSteps, req);

    // Homing
    positionManager.holdPositionDuration(commandedPosition, 0.5);
    positionManager.homing(commandedPosition, currentPosition);
    positionManager.holdPositionDuration(commandedPosition, 0.5);

    // Perform Forward motion
    controller.sendRezeroCommand(500.0f); // sets the current position to 500.0
    commandedPosition = 500.0f;
    currentPosition = 500.0f;
    positionManager.performAcceleration(commandedPosition, currentPosition);
    positionManager.performCruising(commandedPosition, currentPosition);
    positionManager.performDeceleration(commandedPosition, currentPosition);
    positionManager.holdPositionDuration(commandedPosition, 0.5);

    // Perform Reverse motion
    controller.sendRezeroCommand(500.0f); // sets the current position to 500.0
    commandedPosition = 500.0f;
    currentPosition = 500.0f;
    positionManager.performAccelerationReverse(commandedPosition, currentPosition);
    positionManager.performCruisingReverse(commandedPosition, currentPosition);
    positionManager.performDecelerationReverse(commandedPosition, currentPosition);
    positionManager.holdPositionDuration(commandedPosition, 0.5);


//////////////////////////////////////////////////////////////////////////////////////////

    // Stop the motor
    //controller.sendStopCommand();
    controller.closeSerialPort();

    // Graph the torque values collected
    //GraphPlotter plotter;
    //plotter.plot(positionManager.getTorques(), "Torque Readings Through Various Phases");

    return 0;
}
 