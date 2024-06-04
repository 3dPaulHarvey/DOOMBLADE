// CommandGenerator.h
#ifndef COMMANDGENERATOR_H
#define COMMANDGENERATOR_H

#include <vector>
#include <cstring> // For memcpy
#include <iomanip>
#include <sstream>
#include "FloatConverter.h"  // Include the FloatConverter header

class CommandGenerator {
public:
    CommandGenerator(float float1, float float2) {
        // Initialize the command prefix
        //command_prefix = {'c', 'a', 'n', ' ', 's', 'e', 'n', 'd', ' ', '8', '0', '0', '1', ' '};

        // Initialize the command data as hex values
        std::vector<unsigned char> hexData = 
        {0x01, 0x00, 0x0A, // Set Register to Position Mode
        0x0c, 0x02, 0x20}; // float32, 2 of them, starting at 0x20

        //print hexData
        std::cout << "Hex Data Prefix: ";
        for (auto c : hexData) {
            std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(c) << "";
        }
        // Convert and append initial hex data
        command_data = convertToASCII(hexData);

        //print command_data
        std::cout << "\nCommand Data Ascii ";
        for (auto c : command_data) {
            std::cout << c << "";
        }

        // Append converted floats
        appendFloatsToCommandData(float1, float2);

        //floats appended
        std::cout << "\nFloats Appended ";
        for (auto c : command_data) {
            std::cout << c << "";
        }

        // Build and store full command
        //buildAndStoreCommand();

        // Append newline character at the end
        //fullCommand.push_back(0x0A); // Newline character
    }

    std::vector<unsigned char> getCommand() const {
       //return fullCommand;
        return command_data;
    }

private:
    std::vector<unsigned char> command_prefix;
    std::vector<unsigned char> command_data;
    std::vector<unsigned char> fullCommand;

    void appendFloatsToCommandData(float float1, float float2) {
        std::vector<float> floats = {float1, float2};
        for (auto f : floats) {
            auto encodedFloat = FloatConverter::convertFloat(f);
            command_data.insert(command_data.end(), encodedFloat.begin(), encodedFloat.end());
        }
    }

    void buildAndStoreCommand() {
        fullCommand = command_prefix;
        fullCommand.insert(fullCommand.end(), command_data.begin(), command_data.end());
    }

    std::vector<unsigned char> convertToASCII(const std::vector<unsigned char>& hexData) {
        std::vector<unsigned char> asciiData;
        for (auto byte : hexData) {
            asciiData.push_back(nibbleToHexChar((byte >> 4) & 0x0F));
            asciiData.push_back(nibbleToHexChar(byte & 0x0F));
        }
        return asciiData;
    }

    unsigned char nibbleToHexChar(unsigned char nibble) {
        if (nibble < 10) {
            return '0' + nibble;
        } else {
            return 'a' + (nibble - 10);
        }
    }
};

#endif // COMMANDGENERATOR_H
