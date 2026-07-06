#ifndef ARDUINO_SEND_HPP
#define ARDUINO_SEND_HPP

#include <string>
#include <cstdint>

class ArduinoSend {
private:
    int serial_fd; // defines a serial file descriptor that stores a number for the serial port connection between the Jetson and the Arduino.
                   // This is referenced when using write() and read() functions later in the actual code
    std::string port_name; // Allocates space for a string variable that stores the literal directory path of the hardware link on your Jetson (e.g., "/dev/ttyACM0").

public:
    ArduinoSend(const std::string& port = "/dev/ttyACM0"); // class constructor
    ~ArduinoSend(); // destructor ensures that link to linux software is cleanly severed when all operations are finished to prevent errors

    bool initializePort(); 
    bool sendTargetCoordinates(uint16_t id, int16_t  x, int16_t y);
    void closePort();
    void flushCache();
};

#endif
