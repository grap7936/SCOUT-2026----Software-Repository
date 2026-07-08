/////////////////////////////////////////////////////////////
/*
Code Summary: Main execution script to test bidirectional communication between a RaspberryPi/Jetson and an Arduino UNO over USB Serial.
              Generates random test tracking targets, sends them to the Arduino UNO and then sends the results received by the Arduino 
              UNO back to the RaspberryPi/Jetson to verify functionality of the bidirectional communication pathway.

Author: Original test arduino code written in ArduinoIDE by Graeme Appel converted to this format for use in Linux/VNC viewer by gemini

Last Updated: 7/8/2026


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

/////////////////////////////////////////////////////////////

int main() {
    std::string serial_port = "/dev/ttyACM0";  // establish serial connection (likely syntax for the Arduino NANO) to pass into linux system -- could also be "/dev/ttyUSB1" 
                                               // For Arduino UNO it will be /dev/ttyACM0" or /dev/ttyACM1"
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

   // Preallocate necessary variables for the testing loop
    bool exit_testing = false;  // exit testing variable which is only changed to true when inputting the command to enter tracking mode (Y = -5)
    int test_code;
    
    while(exit_testing == false) { // this causes each testing case to be tried as many times as needed until switching to TRACKING MODE each time where then it breaks our of the testing switch cases

    std::cout << "[SYSTEM READY] Sequence active.\n" << std::endl;
    std::cout << "Input desired test code and then press ENTER: ";
    
        // type check wrapper here to stop terminal 
        // string inputs (letters) from triggering infinite input loops
        if (!(std::cin >> test_code)) {
            std::cout << "[ERROR] Invalid input type. Please enter a number." << std::endl;
            std::cin.clear();
            std::cin.ignore(256, '\n');
            continue; 
        }

    switch(test_code) {

    case -1: { // Gimbal-Camera System Sentry Mode

        // std::cout << "Execute Gimbal Sentry Test Sequence (Y = -1).";
        // Note: the command parameter input (input 2) is in rotations per minute that the motor will move at a constant velocity in sentry mode.

    // Uncomment these lines if you want to enable a specific RPM user input rather than some globally defined speed value
        // std::cout << "Input Constant/Double Velocity in [RPM] to spin the system in Sentry mode then press ENTER.";
        // int Sentry_RPM;
        // std::cin >> Sentry_RPM;
        sender.sendTargetCoordinates(0, 0, -1); // replace 0 with Sentry_RPM if you want to use the user input value from above

        std::cout << "Testing will continue unless user leaves TESTING MODE (i.e Y = -5). ONLY LEAVE TESTING MODE WHEN YOU ARE SURE TESTING IS FINISHED. Testing mode can be re-entered with Y = -6 \n\n";

        displayCommandMenu(); // redisplay available commands each time

        break;
    }

    case -2: {  // Test Motor Function by moving motor to a specific degree measurement

        // std::cout << "Execute Motor Test (Y = -2).";

        std::cout << "Input Constant/Double Angle in [deg] to move the motor a specific amount then press ENTER. ";
        int Motor_deg; 
        std::cin >> Motor_deg;
        sender.sendTargetCoordinates(0, Motor_deg, -2); // 2nd input is the amount that the motor will move in degrees from its starting position

        // sendTargetCoordinates() intentionally SKIPS its own read when y == -2, leaving the Arduino's bare-float shaft angle in the serial buffer for
        // readMotorPosition() to consume here. If this stays commented out, that float lingers in the buffer and corrupts the next packet's response read.
        std::cout << "[TELEMETRY CHECK] Querying current shaft angle..." << std::endl;
        double shaft_angle = sender.readMotorPosition();
        std::cout << "[TELEMETRY] Reported shaft angle (rad): " << shaft_angle << std::endl;
        
        std::cout << "Testing will continue unless user leaves TESTING MODE (i.e Y = -5). ONLY LEAVE TESTING MODE WHEN YOU ARE SURE TESTING IS FINISHED. Testing mode can be re-entered with Y = -6\n\n";

        displayCommandMenu(); // redisplay available commands each time


        break;
    }

    case -3: {  // Ping function to test bilateral communications
   
        // std::cout << "Execute Communications Ping Test (Y = -3).";
        std::cout << "Input Number/Byte to receive back to console to check comms then press ENTER. ";
        int byte_value;
        std::cin >> byte_value;
        sender.sendTargetCoordinates(0, byte_value, -3); // sending 25 as an arbitrary returned parameter to test functionality

        std::cout << "Testing will continue unless user leaves TESTING MODE (i.e Y = -5). ONLY LEAVE TESTING MODE WHEN YOU ARE SURE TESTING IS FINISHED. Testing mode can be re-entered with Y = -6\n\n";

        displayCommandMenu(); // redisplay available commands each time


        break;
    } 

    case -4: { // PID controller function -- will be input in ArduinoReceive once I get it from Mansi  
    
        // std::cout << "Execute PID Controller Function Test (Y = -4).";
        sender.sendTargetCoordinates(0, 0, -4); // other two inputs besides -3 are assigned to 0 for now, these will likely have to be altered later depending on necessary inputs of Mansi's code.

        std::cout << "Testing will continue unless user leaves TESTING MODE (i.e Y = -5). ONLY LEAVE TESTING MODE WHEN YOU ARE SURE TESTING IS FINISHED. Testing mode can be re-entered with Y = -6\n\n";

        displayCommandMenu(); // redisplay available commands each time

        break;
    }
    
    case -5: { // Move from testing mode to tracking mode

        // std::cout << "Advance into tracking pipeline (Y = -5).";
        sender.sendTargetCoordinates(0, 0, -5);

        exit_testing = true; // make exit_testing true to move to TRACKING MODE

        break;
    }

    // ******************* Moving back to Testing mode will exist in the live tracking loop once a script integrates testing 
    // and tracking sending modes together -- for now this just has the testing mode materials.
    // case -6: { // Move from tracking mode to testing mode

    //     // std::cout << "Return to Testing Mode (Y = -6)."
    //     sender.sendTargetCoordinates(0,0,-6);

    //     break;
    // }

default: { // catch-all safety parameter for any incorrect user inputs
                std::cout << "[WARNING] Unrecognized command code.\n\n" << std::endl;
                displayCommandMenu();
                break;
            }


    } // end of switch case looping

} // end of entire while loop for testing mode
    
    //cleanup sequence that runs exactly when exiting testing mode
    std::cout << "Shutting down serial link connection..." << std::endl;
    sender.closePort();
    return 0;
}