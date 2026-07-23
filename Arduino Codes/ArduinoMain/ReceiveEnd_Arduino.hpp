/////////////////////////////////////////////////////////////

// NOTE: Ensure that this file (header file) as well as the ArduinoReceive.cpp file and the Main file are in the same folder before sending to the Arduino as otherwise this will not function properly

/*
Code Summary: Contains all the relevant function declarations for each function detailed inside ReceiveEnd_Arduino.cpp and also defines the JetsonPackage structure that stores
x and y position as well as object ID and detecting if a new data packet is available.

Author: Graeme Appel

Last Updated: 7/22/2026
*/


/////////////////////////////////////////////////////////////

// Include Necessary libraries and includes
#ifndef ARDUINO_RECEIVE_H
#define ARDUINO_RECEIVE_H

#include <Arduino.h>

/////////////////////////////////////////////////////////////

// Struct and Class Declarations

/////////////////////////////////////////////////////////////

// --- Custom PID Data Structure ---
/* 1.) customPID struct 

Function elements/inputs:
// --- Tuning & System Constants ---
float Kp; == Proportional Gain: Immediate response to current error. Provides immediate output proportional to how far the motor is from its setpoint.
float Ki; == Integral Gain: Eliminates steady-state position offsets. Multiplies accumulated past error. Eliminates steady-state error over time.
float Kd; == Derivative Gain: Dampens oscillations and overshooting. Multiplies the rate of change of error. Acts as a "dampener" or brake to prevent overshoot and oscillations.
float Kaw; == Anti-Windup Gain: Bleeds off integrator accumulation during saturation. Controls back-calculation anti-windup. When the output saturates (hits limits), 
              Kaw bleeds off the accumulated integral memory to prevent motor overshoot when recovering from saturation.
float T_C; == Derivative Low-Pass Filter Time Constant [seconds] (smooths sensor noise)
float T; == Loop Sample Time / Period dt [seconds]
float max; == Maximum allowed output saturation limit/voltage [ Volts]
float min; == Minimum allowed output saturation limit/voltage [Volts]
float max_rate; == Maximum allowed output slew rate in [Volts/sec]. Prevents extreme voltage spikes that can jerk mechanical systems or blow drivers.

// --- Historical State Variables (Preserved across loop calls) ---
float integral; == stores the running sum of integrated error across loop iterations.
float err_prev; == Tracking error from previous iteration e[k-1]
float deriv_prev; == Low-pass filtered derivative output from previous iteration
float command_sat_prev; == Saturated & slew-rate limited command output from previous iteration u_sat[k-1]
float command_prev; == Raw unsaturated command output from previous iteration u[k-1]


*/

/////////////////////////////////////////////////////////////

// PID data type that contains all necessary information for the PID_step function in ArduinoMain
struct PID {
  float Kp, Ki, Kd, Kaw, T_C, T, max, min, max_rate;
  float integral, err_prev, deriv_prev, command_sat_prev, command_prev;
};

float PID_Step(struct PID *pid, float measurement, float setpoint);

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
    void testPID(int16_t x_coord_offset);
    void write(uint16_t FRAME_NUM, float CURRENT_MOTOR_POS);
    float getRadDiff(float Frame_width_px, float Frame_width_rads, int16_t target_x);


};


// Make the object available to Main.ino
extern ArduinoReceiveClass ArduinoReceive;

#endif

