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
/////////////////////////////////////////////////////////////////////////////////////////////////////////  
/////////////////////////////////////////////////////////////////////////////////////////////////////////  
//////////Initialization  
    std::vector<float> torques;
    const float startPosition = 500.0f;
    const float cruisingEndPosition = 497.0f;  //496.0f  0.6 gear
    const float cruisingReverseEndPosition = 501.8f; // 503.8f 0.6 gear
    const float stepsToAccelerate = 30.0f;  
    const float decelerationSteps = 10.0f; 
    const float maxSpeed = 0.005f; //0.065      // Max speed in units per control loop iteration
    const float decelerationPerStep = (maxSpeed / stepsToAccelerate);
    std::cout << "Deceleration per step: " << decelerationPerStep << std::endl;
    std::cout << "Deceleration per step x 20 setps: " << decelerationPerStep*20.0f << std::endl;

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
    float safetyHoldPosition = 500.0f;
    float extendHoldPosition = 500.0f;
    float positionAverage = 500.0f;


    // SYSTEM STATE MACHINE
    while (true) {

        //SAFETY CHECK
        safety_ok = safetySwitch.readValue(); // Synchronously check the safety button
        if (!safety_ok) {
            ///////////////////////////////////////////////////// //Graph the torque values collected
            GraphPlotter plotter;
            plotter.plot(positionManager.getTorques(), "Torque Readings Through Various Phases");
            return 0;
            state = MotorState::SafetyLockout;
            //break;
        }
        //INITIAL
        if (state == MotorState::Initial) {
            std::cout << "Initial state. Checking system status..." << std::endl;
            nanosleep(&req, NULL);
            if (homeLimitSwitch.readValue() == 0) {  ///////////////// 0
                state = MotorState::Homing;
            } else {
                state = MotorState::WaitingToHome;
            }
        }
        //WAITING TO HOME
        else if (state == MotorState::WaitingToHome) {
            //std::cout << "Waiting to home..." << std::endl;
            positionManager.holdPosition(commandedPosition);
            if (activateSwitch.readValue() == 0) {
                state = MotorState::Homing;
            }
            if (homeLimitSwitch.readValue() == 0) {
                state = MotorState::Homing;
            }
        }
        //HOMING
        else if (state == MotorState::Homing) {
            std::cout << "Homing..." << std::endl;
            controller.sendStopCommand();  //gets controller to a known state
            controller.sendRezeroCommand(500.0f); // sets the current position to 500.0
            commandedPosition = 500.0f;
            homingSuccess = positionManager.homing(commandedPosition, currentPosition);
            if (homingSuccess) {
                state = MotorState::Homed;
            } else {
                state = MotorState::WaitingToHome;
            }
        }
        //HOMED
        else if (state == MotorState::Homed) {
            std::cout << "System is homed, ready for next command." << std::endl;
            state = MotorState::WaitingToExtend;
        }
        //WAITING TO EXTEND
        else if (state == MotorState::WaitingToExtend) {
            //std::cout << "Waiting to extend..." << std::endl;
            positionManager.holdPosition(currentPosition);
            if (activateSwitch.readValue() == 0) {
                state = MotorState::Extending;
            }
        }
        //EXTENDING
        else if (state == MotorState::Extending) {  
            std::cout << "Extending..." << std::endl;
            // Perform Forward motion
            controller.sendRezeroCommand(500.0f); // sets the current position to 500.0
            commandedPosition = 500.0f;
            currentPosition = 500.0f;
            positionManager.performAcceleration(commandedPosition, currentPosition);
            positionManager.performCruising(commandedPosition, currentPosition);
            extendHoldPosition = positionManager.performDeceleration(commandedPosition, currentPosition);
            positionAverage = (commandedPosition + currentPosition) / 2.0f;
            positionManager.holdPositionDuration(positionAverage, 1.0f);
            std::cout << commandedPosition << "\t" << currentPosition << "\t" << positionManager.tripleQuery() << std::endl;
            controller.sendRezeroCommand(500.0f); // sets the current position to 500.0
            commandedPosition = 500.0f;
            currentPosition = 500.0f;
            positionManager.holdPositionDuration(commandedPosition, 0.5f);

            // if (extendLimitSwitch.readValue() == 0) {
            //     state = MotorState::Extended;
            // }else{
            //     state = MotorState::ExtendError;
            // }
            state = MotorState::Extended;

            ///////////////////////////////////////////////////////// //Graph the torque values collected
            // GraphPlotter plotter;
            // plotter.plot(positionManager.getTorques(), "Torque Readings Through Various Phases");
            // return 0;
        }
        //EXTENDED
        else if (state == MotorState::Extended) {
            // std::cout << "System has been successfully extended." << std::endl;
            positionManager.holdPosition(commandedPosition);
            if (activateSwitch.readValue() == 0) {
                state = MotorState::Sheathing;
            }
        }
        //SHEATHING
        else if (state == MotorState::Sheathing) {
            std::cout << "Sheathing..." << std::endl;
            // Perform Reverse motion
            controller.sendRezeroCommand(500.0f); // sets the current position to 500.0
            commandedPosition = 500.0f;
            currentPosition = 500.0f;
            positionManager.performAccelerationReverse(commandedPosition, currentPosition);
            positionManager.performCruisingReverse(commandedPosition, currentPosition);

            /////////////////////////////////////////////////////// //Graph the torque values collected
            // GraphPlotter plotter;
            // plotter.plot(positionManager.getTorques(), "Torque Readings Through Various Phases");
            // return 0;

            positionManager.performDecelerationReverse(commandedPosition, currentPosition);
            positionAverage = (commandedPosition + currentPosition) / 2.0f;
            positionManager.holdPositionDuration(positionAverage, 0.5f);
            std::cout << commandedPosition << "\t" << currentPosition << "\t" << positionManager.tripleQuery() << std::endl;
            if (obstruction_encountered) {
                state = MotorState::WaitingToHome;
            } else {
                state = MotorState::Homing;
            }

        }
        //EXTEND ERROR
        else if (state == MotorState::ExtendError) {
            std::cout << "Extension error: extend limit switch was not activated." << std::endl;
            nanosleep(&req2, NULL);
            state = MotorState::Initial;
        }
        //SAFETY LOCKOUT
        else if (state == MotorState::SafetyLockout) {
            std::cout << "SAFETY LOCKOUT - The system is in a locked state until the safety button is engaged." << std::endl;
            // Wait in safety lockout state until the safety button is pressed
            safetyHoldPosition = positionManager.tripleQuery();
          
                if (safetySwitch.readValue() == 1) {
                    state = MotorState::Initial;  // Reset to initial state
                }
                positionManager.holdPosition(commandedPosition);
       
            
            

        }

    }// END While


/////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Stop the motor
    //controller.sendStopCommand();
    controller.closeSerialPort();

    // Graph the torque values collected
    //GraphPlotter plotter;
    //plotter.plot(positionManager.getTorques(), "Torque Readings Through Various Phases");

    return 0;
}
 