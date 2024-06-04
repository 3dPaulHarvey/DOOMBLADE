#include "MyController.h"
#include "GraphPlotter.h"
#include "MyGpio.h"
#include "PositionManager.h"

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
        std::cerr << "Failed to initialize GPIO button" << std::endl;
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
/////////////////////////////////////////////////////////////////////////////////////////////////////////  

    struct timespec req2 = {0, 5 * 1000};
    MotorState state = MotorState::Initial;
    bool homingSuccess = false;
    bool obstruction_encountered = false;
    bool safety_ok = false;

    while (true) {

        safety_ok = safetySwitch.readValue(); // Synchronously check the safety button
        if (!safety_ok) {
            state = MotorState::SafetyLockout;
        }
        if (state == MotorState::Initial) {
            std::cout << "Initial state. Checking system status..." << std::endl;
            nanosleep(&req, NULL);
            if (homeLimitSwitch.readValue() == 0) {
                state = MotorState::Homing;
            } else {
                state = MotorState::WaitingToHome;
            }
        }
        else if (state == MotorState::WaitingToHome) {
            std::cout << "Waiting to home..." << std::endl;
            positionManager.holdPosition(commandedPosition);
            if (activateSwitch.readValue() == 0) {
                state = MotorState::Homing;
            }
        }
        else if (state == MotorState::Homing) {
            std::cout << "Homing..." << std::endl;
            homingSuccess = positionManager.homing(commandedPosition, currentPosition);
            positionManager.holdPositionDuration(commandedPosition, 0.5);
            if (homingSuccess) {
                state = MotorState::Homed;
            } else {
                state = MotorState::WaitingToHome;
            }
        }
        else if (state == MotorState::Homed) {
            std::cout << "System is homed, ready for next command." << std::endl;
            state = MotorState::WaitingToExtend;
        }
        else if (state == MotorState::WaitingToExtend) {
            std::cout << "Waiting to extend..." << std::endl;
            positionManager.holdPosition(commandedPosition);
            if (activateSwitch.readValue() == 0) {
                state = MotorState::Extending;
            }
        }
        else if (state == MotorState::Extending) {
            std::cout << "Extending..." << std::endl;
            // Perform Forward motion
            controller.sendRezeroCommand(500.0f); // sets the current position to 500.0
            commandedPosition = 500.0f;
            currentPosition = 500.0f;
            positionManager.performAcceleration(commandedPosition, currentPosition);
            positionManager.performCruising(commandedPosition, currentPosition);
            positionManager.performDeceleration(commandedPosition, currentPosition);
            positionManager.holdPositionDuration(commandedPosition, 0.5);
            state = MotorState::Extended;
        }
        else if (state == MotorState::Extended) {
            std::cout << "System has been successfully extended." << std::endl;
            positionManager.holdPosition(commandedPosition);
            if (activateSwitch.readValue() == 0) {
                state = MotorState::Sheathing;
            }
        }
        else if (state == MotorState::Sheathing) {
            std::cout << "Sheathing..." << std::endl;
            // Perform Reverse motion
            controller.sendRezeroCommand(500.0f); // sets the current position to 500.0
            commandedPosition = 500.0f;
            currentPosition = 500.0f;
            positionManager.performAccelerationReverse(commandedPosition, currentPosition);
            positionManager.performCruisingReverse(commandedPosition, currentPosition);
            positionManager.performDecelerationReverse(commandedPosition, currentPosition);
            //positionManager.holdPositionDuration(commandedPosition, 0.5);

            if (obstruction_encountered) {
                state = MotorState::WaitingToHome;
            } else {
                state = MotorState::Homing;
            }
        }
        
        else if (state == MotorState::ExtendError) {
            std::cout << "Extension error: extend limit switch was not activated." << std::endl;
            nanosleep(&req, NULL);
            state = MotorState::Initial;
        }
        else if (state == MotorState::SafetyLockout) {
            std::cout << "SAFETY LOCKOUT - The system is in a locked state until the safety button is engaged." << std::endl;
            // Wait in safety lockout state until the safety button is pressed
            if (safetySwitch.readValue() == 0) {
                state = MotorState::Initial;  // Reset to initial state
            }
            positionManager.holdPosition(commandedPosition);
            
        }

        // Additional states and their handling can be added here.
    }// END While




/////////////////////////////////////////////////////////////////////////////////////////////////////////
    // // Homing
    // positionManager.holdPositionDuration(commandedPosition, 0.5);
    // positionManager.homing(commandedPosition, currentPosition);
    // positionManager.holdPositionDuration(commandedPosition, 0.5);

    // // Perform Forward motion
    // controller.sendRezeroCommand(500.0f); // sets the current position to 500.0
    // commandedPosition = 500.0f;
    // currentPosition = 500.0f;
    // positionManager.performAcceleration(commandedPosition, currentPosition);
    // positionManager.performCruising(commandedPosition, currentPosition);
    // positionManager.performDeceleration(commandedPosition, currentPosition);
    // positionManager.holdPositionDuration(commandedPosition, 0.5);

    // // Perform Reverse motion
    // controller.sendRezeroCommand(500.0f); // sets the current position to 500.0
    // commandedPosition = 500.0f;
    // currentPosition = 500.0f;
    // positionManager.performAccelerationReverse(commandedPosition, currentPosition);
    // positionManager.performCruisingReverse(commandedPosition, currentPosition);
    // positionManager.performDecelerationReverse(commandedPosition, currentPosition);
    // positionManager.holdPositionDuration(commandedPosition, 0.5);

//////////////////////////////////////////////////////////////////////////////////////////

    // Stop the motor
    //controller.sendStopCommand();
    controller.closeSerialPort();

    // Graph the torque values collected
    //GraphPlotter plotter;
    //plotter.plot(positionManager.getTorques(), "Torque Readings Through Various Phases");

    return 0;
}
 