/////////////////////////////////////////////////////////////
/*
Code Summary:

Author: Zachary Dyre, Graeme Appel (only added read motor portion)

Last Updated: 7/9/2026


*/
/////////////////////////////////////////////////////////////


#include <iostream>
#include "ArduinoSend.hpp"
#include <unistd.h>  // For sleep() and usleep()

void setupArduino(ArduinoSend& sender);

int main() {

    // Set up arduino connection

    // Define the serial port node. On a Raspberry Pi, an Arduino UNO typically populates as "/dev/ttyACM0" or "/dev/ttyACM1".
    std::string serial_port = "/dev/ttyACM0"; 
    
    std::cout << "Initializing ArduinoSend on serial port: " << serial_port << std::endl; // output port connection information to the console
    ArduinoSend sender(serial_port); // create instance of the ArduinoSend function configured with the correct serial port
    
    setupArduino(sender);

    sender.sendTargetCoordinates(-3, -3, -3);

    std::ofstream logFile("motor_position.log", std::ios::app);
    std::vector<double> frame_plus_motor_data = sender.readMotorPosition(logFile);

    std::cout << " Frame number detected: " << frame_plus_motor_data[0] << std::endl;
    std::cout << " Motor Position Detected: " << frame_plus_motor_data[1] << " [radians]\n";

}   

void setupArduino(ArduinoSend& sender) {

    // Establish port connection
    if (!sender.initializePort()) { // sends error message if the port cannot be effectively initialized
        std::cerr << "[FATAL] Could not initialize communication pipeline. Exiting." << std::endl;
        return;
    }

    // Manage Hardware Auto-Reset
    // Opening the port forces the Arduino UNO to reset. We must sleep here to give the bootloader time to finish before sending data frames.
    std::cout << "Serial pipeline established. Waiting 2 seconds for Arduino UNO boot sequence." << std::endl;
    sleep(2); 

    // flush system cache before running 1st instance
    sender.flushCache();

    std::cout << "[SYSTEM READY] Pipeline active. Ready for coordinate injection stream.\n" << std::endl;

}
