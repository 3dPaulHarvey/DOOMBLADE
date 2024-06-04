// FloatConverter.h
#ifndef FLOATCONVERTER_H
#define FLOATCONVERTER_H

#include <vector>
#include <cstring> // For memcpy
#include <iomanip>
#include <sstream>

class FloatConverter {
public:
    // Convert a single float to its full hexadecimal ASCII representation
    static std::vector<unsigned char> convertFloat(float value) {
        std::vector<unsigned char> convertedData;

        // Convert float to bytes
        unsigned char bytes[sizeof(float)];
        std::memcpy(bytes, &value, sizeof(float));

        // Convert each byte to two hex digits in ASCII
        for (int i = 0; i < sizeof(float); i++) {
            convertedData.push_back(toHexDigit((bytes[i] >> 4) & 0x0F));  // High nibble
            convertedData.push_back(toHexDigit(bytes[i] & 0x0F));         // Low nibble
        }
        return convertedData;
    }

private:
    // Helper function to convert a nibble to its hex digit in ASCII
    static unsigned char toHexDigit(unsigned int nibble) {
        return nibble < 10 ? '0' + nibble : 'a' + (nibble - 10);
    }
};

#endif // FLOATCONVERTER_H
