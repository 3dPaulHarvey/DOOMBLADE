#ifndef MY_CONTROLLER_H
#define MY_CONTROLLER_H

#include <iostream>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <chrono>
#include <vector>
#include <sstream>
#include <algorithm>
#include "FloatConverter.h"
#include <limits>

class MyController {
public:
    MyController(const char* portName);
    ~MyController();
    bool setupSerialPort();
    void clearBuffer();
    void sendStopCommand();
    void sendBrakeCommand();
    void sendRezeroCommand(float float1);
    void sendQueryCommand();
    void sendWriteCommand(float float1, float float2);
    void sendWriteOnlyCommand(float float1, float float2);
    void sendCustomCommand();
    std::vector<float> sendReadCommand();
    void closeSerialPort();

private:
    int fd;
    std::string portName;

};

// Constructor
MyController::MyController(const char* portName) : portName(portName), fd(-1) {}

// Destructor
MyController::~MyController() {
    if (fd != -1) {
        closeSerialPort();
    }
}

// SERIAL PORT SETUP
bool MyController::setupSerialPort() {
    fd = open(portName.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        std::cerr << "Error opening " << portName << ": " << strerror(errno) << std::endl;
        return false;
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "Error from tcgetattr: " << strerror(errno) << std::endl;
        close(fd);
        return false;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error from tcsetattr: " << strerror(errno) << std::endl;
        close(fd);
        return false;
    }

    return true;
}

// CLEAR BUFFER
void MyController::clearBuffer() {    
    char buf[256];
    read(fd, buf, sizeof buf);
}

// STOP COMMAND
void MyController::sendStopCommand() {
    std::string message = "can send 0001 010000\n";
    write(fd, message.c_str(), message.length());
}

// BRAKE COMMAND
void MyController::sendBrakeCommand() {
    std::string message = "can send 0001 01000f\n";
    write(fd, message.c_str(), message.length());
}

// WRITE COMMAND
void MyController::sendWriteCommand(float float1, float float2) {
    std::string commandPrefix = "can send 8001 01000a0c0220";
    std::string commandSuffix = "1c0301\n";   //Read 3 registers starting at 0x01
    std::vector<unsigned char> float1Bytes = FloatConverter::convertFloat(float1);
    std::vector<unsigned char> float2Bytes = FloatConverter::convertFloat(float2);
    std::string float1Hex(float1Bytes.begin(), float1Bytes.end());
    std::string float2Hex(float2Bytes.begin(), float2Bytes.end());
    std::string commandData = float1Hex + float2Hex;
    std::string command = commandPrefix + commandData + commandSuffix;
    write(fd, command.c_str(), command.length());
}


// WRITE ONLY COMMAND
void MyController::sendWriteOnlyCommand(float float1, float float2) {
    std::string commandPrefix = "can send 01 01000a0c0220";
    std::string commandSuffix = "\n";   
    std::vector<unsigned char> float1Bytes = FloatConverter::convertFloat(float1);
    std::vector<unsigned char> float2Bytes = FloatConverter::convertFloat(float2);
    std::string float1Hex(float1Bytes.begin(), float1Bytes.end());
    std::string float2Hex(float2Bytes.begin(), float2Bytes.end());
    std::string commandData = float1Hex + float2Hex;
    std::string command = commandPrefix + commandData + commandSuffix;
    //std::cout << "Command: " << command << std::endl;
    write(fd, command.c_str(), command.length());
    usleep(5);
    //clear buffer
    char buf[256];
    read(fd, buf, sizeof buf);
}

// CUSTOM COMMAND
void MyController::sendCustomCommand() {
    std::string commandPrefix = "can send 0001 ";
    std::string commandSuffix = "\n";
    std::string commandData = "0db10200004040";
    std::string command = commandPrefix + commandData + commandSuffix;
    std::cout << "Command: " << command << std::endl;
    write(fd, command.c_str(), command.length());
}

// REZERO 
void MyController::sendRezeroCommand(float float1) {
    std::string commandPrefix = "can send 0001 0db102";
    std::string commandSuffix = "\n";

    std::vector<unsigned char> float1Bytes = FloatConverter::convertFloat(float1);
    std::string float1Hex(float1Bytes.begin(), float1Bytes.end());
    std::string commandData = float1Hex; 
    //std::string commandData = "0db102 00 00 40 40";
    std::string command = commandPrefix + commandData + commandSuffix;
    //std::cout << "Command: " << command << std::endl;
    write(fd, command.c_str(), command.length());
}

// SEND QUERY COMMAND
void MyController::sendQueryCommand() {
    std::string commandPrefix = "can send 8001 ";
    std::string commandSuffix = "1c0301\n";   //Read 3 register starting at 0x01
    std::string command = commandPrefix  + commandSuffix;
    write(fd, command.c_str(), command.length());
}

// HEX STRING TO FLOAT
float hexStringToFloat(const std::vector<std::string>& hexParts) {
    unsigned int x = 0;
    std::stringstream ss;
    for (const auto& part : hexParts) { ss << part; }
    ss >> std::hex >> x; 
    float f;
    std::memcpy(&f, &x, sizeof(x)); // Reinterpret the bits of x as a float
    return f;
}

// Send read command and return vector of floats
std::vector<float> MyController::sendReadCommand() {
    char buf[256];
    read(fd, buf, sizeof buf);

    std::vector<float> floats;

    // Helper lambda to convert hex in buffer to float
    auto convertToFloat = [&](int start) -> float {
        std::vector<std::string> parts;
        for (int j = 0; j < 4; ++j) {
            parts.emplace_back(buf + start + j * 2, 2);
        }
        std::reverse(parts.begin(), parts.end());
        return hexStringToFloat(parts);
    };

    // Read three floats from specified buffer positions
    floats.push_back(convertToFloat(18)); // First float
    floats.push_back(convertToFloat(26)); // Second float
    floats.push_back(convertToFloat(34)); // Third float

    return floats;
}

// Close the serial port
void MyController::closeSerialPort() {
    if (fd != -1) {
        close(fd);
        fd = -1;
    }
}

#endif // MY_CONTROLLER_H
