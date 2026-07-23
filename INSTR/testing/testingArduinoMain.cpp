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
#include <fstream>
#include <vector>
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

    std::string serial_port = "/dev/ttyCH341USB0";  // establish serial connection (likely syntax for the Arduino NANO) to pass into linux system 

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
        std::ofstream logFile("motor_position.log", std::ios::app);
        std::vector<double> frame_plus_motor_data = sender.readMotorPosition(logFile);

        // extracting the individual variables from the vector 
        if (frame_plus_motor_data.size() == 2 && frame_plus_motor_data[0] != -1.0) { // conditions to ensure that data being sent happens while in tracking mode and not during the -1 test case
            int frame_num = static_cast<int>(frame_plus_motor_data[0]); // Extracted Frame Number
            double shaft_angle = frame_plus_motor_data[1];             // Extracted Shaft Angle
            
            std::cout << "[TELEMETRY] Received Frame Number: " << frame_num << std::endl;
            std::cout << "[TELEMETRY] Reported shaft angle in (rad): " << shaft_angle << std::endl;
        } else {
            std::cout << "[TELEMETRY] Error: Telemetry returned empty or failed to parse." << std::endl;
        }
        
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

    case -4: { // PID controller function -- sends x-coordinate displacement to cause the PID_step function to move to target coordinates 3 times to observe overall overshoot and system functionality
    
        // std::cout << "Execute PID Controller Function Test (Y = -4).";
        int target_angle = 45; // target angle of 45 degrees along the x-axis to move the motor using PID control

        for (int i = 0; i < 3; i++) { // loop to send the PID controller 3 different target coordinates to test the functionality of the PID controller and observe overshoot and system response time

            sender.sendTargetCoordinates(0, target_angle, -4); // sends a 45 degree x-coordinate displacement to the PID controller from its current position

            // Force the Jetson to wait 3 seconds to let the gimbal physically swing and settle before sending the next movement coordinate for the PID control.
            sleep(3);

            target_angle = target_angle * -1; // flips the target angle to -45 degrees to move the motor back to its original position using PID control for the next iteration
           }
        

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
                return 0; // exit the program if an unrecognized command is input to prevent any errors from occurring
                break;
            }


    } // end of switch case looping

} // end of entire while loop for testing mode
    
    //cleanup sequence that runs exactly when exiting testing mode
    std::cout << "Shutting down serial link connection..." << std::endl;
    sender.closePort();
    return 0;
}
