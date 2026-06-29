/////////////////////////////////////////////////////////////
/*
Code Summary: Main execution script to test bidirectional communication between a RaspberryPi/Jetson and an Arduino UNO over USB Serial.
              Generates random test tracking targets, sends them to the Arduino UNO and then sends the results received by the Arduino 
              UNO back to the RaspberryPi/Jetson to verify functionality of the bidirectional communication pathway.

Author: Original test arduino code written in ArduinoIDE by Graeme Appel converted to this format for use in Linux/VNC viewer by gemini

Last Updated: 6/29/2026


*/
/////////////////////////////////////////////////////////////

// Necessary libraries and header file includes

#include "ArduinoSend.hpp"
#include <iostream>
#include <unistd.h>

/////////////////////////////////////////////////////////////

// User defined functions:

/* 1.) displayCommandMenu() function

Function description: displays all available test commands and input numbers/codes to activate each different test case.

Inputs: 
None

Outputs:
None


*/

void displayCommandMenu() {
    std::cout << "////////////////////////////////////////////////////////////////////" << std::endl;
    std::cout << "                     SYSTEM COMMAND MAP SYSTEM                      " << std::endl;
    std::cout << "////////////////////////////////////////////////////////////////////" << std::endl;
    std::cout << " Use the following target_Y_coordinate parameters to control testing:" << std::endl;
    std::cout << "  Y = -1  ->  Camera / Gimbal Sentry Mode Test" << std::endl;
    std::cout << "  Y = -2  ->  Single-Axis Gimbal Motor Step Drive Test" << std::endl;
    std::cout << "  Y = -3  ->  Bilateral Communications Loopback Ping Test" << std::endl;
    std::cout << "  Y = -4  ->  PID Controller Function Test" << std::endl;
    std::cout << "  Y = -5  ->  State Transition: Shift to Operational TRACKING_MODE" << std::endl;
    std::cout << "  Y = -6  ->  State Transition: Force Escape back to Safe TEST_MODE" << std::endl;
    std::cout << "  Y >= 0  ->  Active Target Pixel Tracks (Only parsed in TRACKING_MODE)" << std::endl;
    std::cout << "////////////////////////////////////////////////////////////////////\n" << std::endl;
}

/* 2.) waitForUser() function

Function description: Inputs the starting statement for each different test case and allows input from the user to activate each separate test case.

Inputs: 
None

Outputs:
None


*/

void waitForUser(const std::string& stepMessage) {
    std::cout << ">>> " << stepMessage << " Press ENTER to transmit package...";
    std::cin.get(); 
}

/////////////////////////////////////////////////////////////

int main() {
    std::string serial_port = "/dev/ttyUSB0";  // establish serial connection (likely syntax for the Arduino NANO) to pass into linux system -- could also be "/dev/ttyUSB0" 
                                               // -- check this by plug the NANO into your Jetson's USB port, open a terminal, and run-> Bash: ls /dev/tty*

    ArduinoSend sender(serial_port); // create instance of the ArduinoSend function

    // Output the explanatory text block right at launch
    displayCommandMenu();

    if (!sender.initializePort()) { // output error nessage if initializing the serial port is unsuccessful
        std::cerr << "[FATAL] Could not initialize communication pipeline. Exiting." << std::endl;
        return 1;
    }

    std::cout << "Serial pipeline established. Waiting 2 seconds for Arduino UNO boot sequence." << std::endl;
    sleep(2); 

    // flush system cache before running 1st instance
    sender.flushCache(); 

    std::cout << "[SYSTEM READY] Sequence active.\n" << std::endl;

    // Execution steps 

    // Gimbal-Camera System Sentry Mode
    waitForUser("Execute Gimbal Sentry Test Sequence (Y = -1).");
    // Note: the command parameter input (input 2) is in rotations per minute that the motor will move at a constant velocity in sentry mode.
    sender.sendTargetCoordinates(0, 0, -1);

    // Test Motor Function by moving motor to a specific degree measurement
     waitForUser("Execute Motor Test (Y = -2).");
     // Note: the command parameter input (input 2) is in degrees that the motor will move.
    sender.sendTargetCoordinates(0, 25, -2); // 2nd input is the amount that the motor will move in degrees from its starting position

    // Ping function to test bilateral communications
    waitForUser("Execute Communications Ping Test (Y = -3).");
    sender.sendTargetCoordinates(0, 25, -3); // sending 25 as an arbitrary returned parameter to test functionality

    // PID controller function -- will be input in AruinoReceive once I get it from Mansi
    waitForUser("Execute PID Controller Function Test (Y = -4).");
    sender.sendTargetCoordinates(0, 0, -4); // other two inputs besides -3 are assigned to 0 for now, these will likely have to be altered later depending on necessary inputs of Mansi's code.

    // Move from testing mode to tracking mode
    waitForUser("Advance into tracking pipeline (Y = -5).");
    sender.sendTargetCoordinates(0, 0, -5);

    waitForUser("Return to Testing Mode (Y = -6).");
    sender.sendTargetCoordinates(0,0,-6);

    sender.closePort();
    return 0;
}