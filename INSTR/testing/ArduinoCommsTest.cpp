/////////////////////////////////////////////////////////////
/*
Code Summary: Main execution script to test bidirectional communication between a RaspberryPi/Jetson and an Arduino UNO over USB Serial.
              Generates random test tracking targets, sends them to the Arduino UNO and then sends the results received by the Arduino 
              UNO back to the RaspberryPi/Jetson to verify functionality of the bidirectional communication pathway.

Author: Original test arduino code written in ArduinoIDE by Graeme Appel converted to this format for use in Linux/VNC viewer by gemini

Last Updated: 6/24/2026
*/
/////////////////////////////////////////////////////////////

// Necessary Libraries and Header files

#include "ArduinoSend.hpp"
#include <iostream>
#include <unistd.h>  // For sleep() and usleep()
#include <cstdlib>   // For rand() and srand()
#include <ctime>     // For seeding the random number generator

/////////////////////////////////////////////////////////////

int main() {

    // Seed the random number generator for our test target data
    std::srand(std::time(nullptr)); // srand creates an ACTUALLY random (i.e new every time) random number for use in creating the random values for our test targets as opposed to std::rand

    // Define the serial port node. On a Raspberry Pi, an Arduino UNO typically populates as "/dev/ttyACM0" or "/dev/ttyACM1".
    std::string serial_port = "/dev/ttyACM0"; 
    
    std::cout << "Initializing ArduinoSend on serial port: " << serial_port << std::endl; // output port connection information to the console
    ArduinoSend sender(serial_port); // create instance of the ArduinoSend function configured with the correct serial port

    // Establish port connection
    if (!sender.initializePort()) { // sends error message if the port cannot be effectively initialized
        std::cerr << "[FATAL] Could not initialize communication pipeline. Exiting." << std::endl;
        return 1;
    }

    // Manage Hardware Auto-Reset
    // Opening the port forces the Arduino UNO to reset. We must sleep here to give the bootloader time to finish before sending data frames.
    std::cout << "Serial pipeline established. Waiting 2 seconds for Arduino UNO boot sequence." << std::endl;
    sleep(2); 
    std::cout << "[SYSTEM READY] Pipeline active. Beginning coordinate injection stream.\n" << std::endl;

    // Execution Loop
    // Emulates tracking loop execution firing every 3 seconds
    int loop_count = 1;
    while (loop_count <= 5) { // Runs 5 times for verification test
        std::cout << "--- Execution Cycle " << loop_count << " ---" << std::endl;

        // Generate arbitrary mock data fields within valid 16-bit boundaries
        uint16_t target_test_id = std::rand() % 60000;          // 0 to 59999
        int16_t test_X_pos = (std::rand() % 201) - 100;    // -100 to 100
        int16_t test_Y_pos  = (std::rand() % 201) - 100;    // -100 to 100

        // Transmit coordinates down to the UNO and automatically catch the echo
        bool success = sender.sendTargetCoordinates(target_test_id, test_X_pos, test_Y_pos);
        
        if (!success) { // if target coordinates have not been sent (i.e bool success = false) then output an error message to the console
            std::cerr << "[ERROR] Pipeline broken during transmit phase." << std::endl;
        }

        std::cout << "--------------------------------\n" << std::endl;
        
        loop_count++; // increment loop count
        sleep(3); // Wait 3 seconds before processing the next mock target frame
    }

    // When fully finished with all looping, send a message tp the console to show that connection has been severed.
    std::cout << "Test script complete. Cleaning up communication channel." << std::endl;
    return 0;
}