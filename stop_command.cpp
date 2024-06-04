#include <iostream>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

// Function to setup the serial port and return the file descriptor
int setupSerialPort(const char* portName) {
    int fd = open(portName, O_RDWR | O_NOCTTY | O_SYNC); // Open the serial port
    if (fd < 0) {
        std::cerr << "Error opening " << portName << ": " << strerror(errno) << std::endl;
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0) { // Get current serial port settings
        std::cerr << "Error from tcgetattr: " << strerror(errno) << std::endl;
        close(fd);
        return -1;
    }

    cfsetospeed(&tty, B115200); // Set baud rate to 115200
    cfsetispeed(&tty, B115200);

    tty.c_cflag |= (CLOCAL | CREAD);  // Enable the receiver and set local mode
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;      // 8-bit characters
    tty.c_cflag &= ~PARENB;  // No parity bit
    tty.c_cflag &= ~CSTOPB;  // 1 stop bit
    tty.c_cflag &= ~CRTSCTS; // No hardware flow control

    // Make raw
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_oflag &= ~OPOST;
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_cflag &= ~(CSIZE | PARENB);
    tty.c_cflag |= CS8;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) { // Apply the settings immediately
        std::cerr << "Error from tcsetattr: " << strerror(errno) << std::endl;
        close(fd);
        return -1;
    }

    return fd;
}

int main() {
    const char* portName = "/dev/fdcanusb"; // Replace with your serial port name
    int fd = setupSerialPort(portName);
    if (fd < 0) return 1;

    // The message to send
    std::string message = "can send 0001 010000\n";

    // Write the message to the serial port
    if (write(fd, message.c_str(), message.length()) < 0) {
        std::cerr << "Error writing to serial port: " << strerror(errno) << std::endl;
        close(fd);
        return 1;
    }

    std::cout << "Message sent: " << message << std::endl;

    close(fd); // Close the serial port
    return 0;
}
