/////////////////////////////////////////////////////////////

// NOTE: Ensure that this file (header file) as well as the ArduinoReceive.cpp file and the Main file are in the same folder before sending to the Arduino as otherwise this will not function properly

/*
Code Summary:  This code defines all of the separate user defined functions necessary for initializing connection between the Jetson and Arduino; receiving and parsing data sent
 from the Jetson to the Arduino; testing various system components; and writing pertinent target data back to the Jetson.

Author: Graeme Appel

Last Updated: 7/13/2026
*/


/////////////////////////////////////////////////////////////

// Include Necessary libraries and includes
#include "ReceiveEnd_Arduino.hpp"
#include <SimpleFOC.h>

/////////////////////////////////////////////////////////////

// Other Setup and pre-allocations 

// Gain external access to the hardware components and targets defined in Main.ino
extern BLDCMotor motor;
extern float motor_target_angle;
extern float motor_target_velocity;

// Create Instance of the ArduinoReceive Class to handle all incomoing data
ArduinoReceiveClass ArduinoReceive;

// Function Definitions for the ArduinoReceive class

/////////////////////////////////////////////////////////////

/* 1.) begin() function

Function description: initializes the serial UART connection between the Jetson and the Arduino.

Inputs: 
1.) unsigned long baudrate: the baud rate for the serial connection. Must perfectly match the B115200 set in the Jetson code.

Outputs:
None


*/


/////////////////////////////////////////////////////////////

void ArduinoReceiveClass::begin(unsigned long baudrate) {
    Serial.begin(baudrate);
}

/////////////////////////////////////////////////////////////

/* 2.) read() function

Function description: This function checks for incoming data from the Jetson and unpacks it into a JetsonPackage struct. 
It reads the data in a specific format, ensuring that the data is valid and complete before returning it.

Inputs: 
None

Outputs:
1.) JetsonPackage: A struct containing the unpacked data from the Jetson, including x and y coordinates, an ID, and a flag indicating whether new data is available.


*/


/////////////////////////////////////////////////////////////

JetsonPackage ArduinoReceiveClass::read() { // structure containing object ID, x and y position and a boolean variable stating if there is a new data packet present

    JetsonPackage Data_Package = {0, 0, 0, false}; // before any data packets have been read in this must be initialized as 0's and false and then after data is read in this will be altered

    if (Serial.available() >= 8) { // ensures there are equal to or more than 8 bytes free meaning that there is at least 1 full data packet to be sent over
        if (Serial.read() == '!') { // starting character check

            // Read in each 8 bit segment of the 16 bit x and y coordinates and 16 bit target ID for each target. This is done in the same order described in the sending side in ArduinoSend.cpp.
            uint8_t id_high = Serial.read();
            uint8_t id_low = Serial.read();
            uint8_t x_high = Serial.read(); 
            uint8_t x_low = Serial.read();
            uint8_t y_high = Serial.read();
            uint8_t y_low = Serial.read();
            uint8_t end_marker = Serial.read();

            // If at the end of a given data packet sent over, assign and reassemble each 8 bit data packet into the original, full 16 bit data packages.
            if (end_marker == '\n') { // end marker check

                Data_Package.id = (uint16_t)((id_high << 8) | id_low);
                Data_Package.x  = (int16_t)((x_high << 8) | x_low);
                Data_Package.y  = (int16_t)((y_high << 8) | y_low);

                Data_Package.hasNewData = true; // after finishing processing 1 data packet, turn this variable to true so another data packet is processed.
            } 
            
            else {
                while (Serial.available() > 0 && Serial.read() != '\n');
            }

        } 
        else {
            while (Serial.available() > 0 && Serial.read() != '\n');
        }
    }
    return Data_Package;
}

/////////////////////////////////////////////////////////////

/* 3.) motor_test() function

Function description: This function is used to test the motor functionality by setting moving the motor forward 1 full revolution
from its starting position and outputting the final angle back to the Jetson.

Inputs: 
None

Outputs:
None

*/


/////////////////////////////////////////////////////////////


void ArduinoReceiveClass::motor_test() {

    motor.controller = MotionControlType::angle; // set to angle control mode
    motor_target_angle = motor.shaft_angle + (2 * PI); // move forward the current motor angle one rotation (360 degrees forward)
    
    delay(10); // Let the motor settle slightly
    Serial.println(motor.shaft_angle, 4); // Report position
}

/////////////////////////////////////////////////////////////

/* 4.) ping_bilateral_comms() function

Function description: This function is used to test the bilateral communications functionality by sending a test byte from the Arduino to the Jetson

Inputs: 
1.) int16_t parameter: The test byte value to be sent back to the Jetson for verification.

Outputs:
None

*/


/////////////////////////////////////////////////////////////

void ArduinoReceiveClass::ping_bilateral_comms(int16_t parameter) {
    Serial.println(F("COMMAND: Bilateral Communications Connection testing engaged"));
    Serial.print(F("Test Byte Value received: ")); Serial.println(parameter);
}

/////////////////////////////////////////////////////////////

/* 5.) write() function

Function description: Takes in the Frame Number of the camera since each interrupt and the current motor position and outputs them back to the Jetson so that they can be saved to a text file.

Inputs: 
1.) uint16_t FRAME_NUM: The frame number of the camera since each interrupt.
2.) float CURRENT_MOTOR_POS: The current motor position.

Outputs:
None

*/


/////////////////////////////////////////////////////////////


void ArduinoReceiveClass::write(uint16_t FRAME_NUM, float CURRENT_MOTOR_POS) {

    // Structured confirmation telemetry pipeline back down to the Jetson
    Serial.print(F("Frame ["));
    Serial.print(FRAME_NUM);
    Serial.print(F("] Pos: "));
    Serial.println(CURRENT_MOTOR_POS, 4);


};

