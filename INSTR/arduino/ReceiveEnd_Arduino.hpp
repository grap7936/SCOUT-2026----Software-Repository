/////////////////////////////////////////////////////////////

// NOTE: Ensure that this file (header file) as well as the ArduinoReceive.cpp file and the Main file are in the same folder before sending to the Arduino as otherwise this will not function properly

/*
Code Summary: Contains all the relevant function declarations for each function detailed inside ReceiveEnd_Arduino.cpp and also defines the JetsonPackage structure that stores
x and y position as well as object ID and detecting if a new data packet is available.

Author: Graeme Appel

Last Updated: 7/14/2026
*/


/////////////////////////////////////////////////////////////

// Include Necessary libraries and includes
#ifndef ARDUINO_RECEIVE_H
#define ARDUINO_RECEIVE_H

#include <Arduino.h>

/////////////////////////////////////////////////////////////

// Struct and Class Declarations

// Struct that contains the data packages that the Jetson will send over to the Arduino to be processed
struct JetsonPackage {

    int16_t x;        // x-coordinate
    int16_t y;        // y-coordinate
    uint16_t id;      // target/object ID
    bool hasNewData;  // sets whether or not coordinate data is being read in after each data packet
};


// Define ArduinoReceive Class and its associated member functions
class ArduinoReceiveClass {

public:

    // Define all Class member functions
    void begin(unsigned long baudrate); // the begin function initializes the Serial UART connection. Must perfectly match the B115200 
    void startSentryMode();
    JetsonPackage read(); // defines JetsonPackage function to checking and unpack all data coming from the Jetson
    void motor_test();
    void ping_bilateral_comms(int16_t parameter);
    void write(uint16_t FRAME_NUM, float CURRENT_MOTOR_POS);
    float getRadDiff(float Frame_width_px, float Frame_width_rads, int16_t target_x);

};


// Make the object available to Main.ino
extern ArduinoReceiveClass ArduinoReceive;

#endif

