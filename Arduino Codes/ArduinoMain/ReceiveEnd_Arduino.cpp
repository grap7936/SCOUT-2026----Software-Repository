/////////////////////////////////////////////////////////////

// NOTE: Ensure that this file (header file) as well as the ArduinoReceive.cpp file and the Main file are in the same folder before sending to the Arduino as otherwise this will not function properly

/*
Code Summary:  This code defines all of the separate user defined functions necessary for initializing connection between the Jetson and Arduino; receiving and parsing data sent
from the Jetson to the Arduino; testing various system components; and writing pertinent target data back to the Jetson.

Author: Graeme Appel

Last Updated: 7/22/2026
*/


/////////////////////////////////////////////////////////////

// Include Necessary libraries and includes
#include "ReceiveEnd_Arduino.hpp"
#include <SimpleFOC.h>

/////////////////////////////////////////////////////////////

// Other Setup and pre-allocations 

// Gain external access to the hardware components and targets defined in Main.ino
extern BLDCMotor motor; // reference external motor encoder object defined in the main script
extern float motor_target_angle; // reference motor necessary motor parameters
extern float motor_target_velocity;
extern volatile int FRAME_NUM; // reference necessary FRAME_NUM from main script
extern const float Sentry_RPM;
extern const float ANGLE_THRESHOLD_RAD; // reference necessary PID function parameters
extern const float VELOCITY_THRESHOLD_RAD_S; // CHANGED: Brought in externally for testPID loop
extern PID motor_pid; // PID object reference
extern MagneticSensorPWM sensor; // Brought in externally to grab current_velocity

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

/* 2.) startSentryMode() function

Function description: Moves the motor to velocity control rather than angle control and begins to move the motor in circles with a constant RPM. 
 speed that is input as a global variable to the main script as Sentry_RPM. The RPM input is converted to rad/s for motor_target_velocity.

Inputs: 
None

Outputs:
None


*/

/////////////////////////////////////////////////////////////

void ArduinoReceiveClass::startSentryMode() {

     motor.controller = MotionControlType::velocity;
    motor_target_velocity = Sentry_RPM * (2.0 * PI / 60.0); // converts RPM input for Sentry_RPM into rad/s for motor_target_velocity.

}








/////////////////////////////////////////////////////////////

/* 3.) read() function

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

/* 4.) motor_test() function

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

    Serial.print(FRAME_NUM); // Accesses the current frame number
    Serial.print(",");
    Serial.println(motor.shaft_angle, 4); // Uses the live shaft angle, NOT the latched interrupt position NOTE: this line is different from that in the write function because
                                          // motor.shaft_angle is the measurement of wher ethe motor is immediately after a testing movement whereas CURRENT_MOTOR_POS logs the position immediately after the camera shutter clicks.
}

/////////////////////////////////////////////////////////////

/* 5.) ping_bilateral_comms() function

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

/* 6.) testPID() function

Function description: Received large x-coordinate displacements from the ArduinoSend/testingArduinoMain side and mactivates the PID system to measure
                      the overshoot each time after the PID control system is initiated. (This will be called 2 or 3 times in total as the send side 
                      will send 3 separate x-coordinates with delay in the middle to allow for movement). This is done by sending a large x-coordinate 
                      offset from the ArduinoSend/testingArduinoMain scripts side to the receive and PID_step function side so that the PID_step
                      function can create new setpoints far away from where the camera is currently pointed. This will cause the gimbal-camera 
                      system to swivel back and forth to the +/- of the new setpoint sent to analyze the overshoot demonstrated by the PID_step
                      function configuration parameters.

Inputs: 
1.) int16_t x_coord_offset == an x-coordinate send from the testingArduinoMain/ArduinoSend side of the code and this offset value is used by the PID function to set
 a new setpoint in either direction from where the gimbal-camera system is currently pointed. 

Outputs:
None

*/


/////////////////////////////////////////////////////////////

void ArduinoReceiveClass::testPID(int16_t x_coord_offset) {

// Change Motion control type to torque to allow for control_voltage inputs to the move motor function for PID control
motor.controller = MotionControlType::torque;

float rad_offset = x_coord_offset * (PI / 180.0); // convert degree input from the testingArduinoSend side into [rads]

// Set motor setpoint by adding the x-coordinate offset to the current motor position for input into the PID_step function
float setpoint = motor.shaft_angle + rad_offset;

float angle_error = fabs(setpoint - motor.shaft_angle);
float current_velocity = fabs(sensor.getVelocity()); //  get real velocity of the PWM sensor linked to the motor

unsigned long test_start = millis(); //  Added test timer
unsigned long last_time = micros();  // Local static-equivalent definition to fix the zero dt bug on first iteration

// Call PID_step function continuously to perform desired motor movmement command until desired x-coordinate position has been reached
while (((angle_error > ANGLE_THRESHOLD_RAD) || (current_velocity > VELOCITY_THRESHOLD_RAD_S)) && (millis() - test_start) < 2000) {

    motor.loopFOC(); // call this to enable automatic motor movement when global variables change for each loop iteration so the motor does not stall or fail

    // Calculate the sample time dt for input into the PID_step function call
    unsigned long now = micros();  // stores internal microsecond clock of Arduino Processor to variable clocking when "Now" is in time
    float dt = (now - last_time) / 1000000.0f; // converts microsecond difference to seconds
    last_time = now; // prepares last_time for the next loop pass so that the next calculated dt measures only the time between successive iterations.
    if (dt <= 0.0f || dt > 0.1f) dt = 0.001f; // if the dt step is negative or greater than 0.1 seconds then it is not a valid window for the PID and 
                                              // Arduino to process so it forces the time step them to be forced back to 0.001 or 1 millisecond.
    motor_pid.T = dt; // assigns time step value to the time field of the motor_pid struct object

    // Determine control voltage necessary to move the gimbal-motor system to the desired input coordinate
    float control_voltage = PID_Step(&motor_pid, motor.shaft_angle, setpoint);
    motor.move(control_voltage); // move motor using calculated control voltage

    // Update the angle_error and current velocity to the new motor parameters to see if the voltage input took the motor within the desired target range -- otherwise runs the loop again with new error
    angle_error = fabs(setpoint - motor.shaft_angle);
    current_velocity = fabs(sensor.getVelocity());


}

// Reset motor and PID states when arriving to the desired target coordinate to be able to be setup for the next x-coordinate sent from the testingArduinoMain script
motor.move(0); // Zero output voltage

// Reset PID states to prevent windup
motor_pid.integral = 0;
motor_pid.command_prev = 0;
motor_pid.command_sat_prev = 0;
motor_pid.err_prev = 0;
motor_pid.deriv_prev = 0;

}






/////////////////////////////////////////////////////////////

/* 7.) write() function

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
    Serial.print(FRAME_NUM);
    Serial.print(",");
    Serial.println(CURRENT_MOTOR_POS, 4);


};

/////////////////////////////////////////////////////////////

/* 8.) getRadDiff() function

Function description: Takes in the camera's frame width in both pixels and radians. Applies basic math to get the radians per pixel and center coordinate of the camera frame.
 After getting this, the function then multiplies it by radians per pixel to get the radians difference. This radians difference will later be passed to the PID loop which will 
 be passed top the motor to know how much to move the motor to point at the target.

Inputs: 
1.) double Frame_width_px == given camera frame width in pixels -- specified as a global variable in the ArduinoMain.ino code
2.) double Frame_width_rads == given camera frame width in radians -- specified as a global variable in the ArduinoMain.ino code
3.) int16_t target_x == x position of any given target as stored by the ArduinoReceive side in JetsonPackage.x for each separate object

Outputs:
1.) double rad_diff == difference in radians between the center of the frame of the camera and the target identified. This will likely be the x-dimension of the camera frame as currently the gimbal motor system can only move in one dimension.

*/


/////////////////////////////////////////////////////////////

float ArduinoReceiveClass::getRadDiff(float Frame_width_px, float Frame_width_rads, int16_t target_x) {

    // If there are negative x-inputs this means there are non-real inputs and so if this is the case, we want the camera/gimbal system to default back to sentry mode and return 0 for the rad_diff
    if (target_x <= 0) {  
       startSentryMode();
       return 0.0f;
    };

    float rads_per_pixel = (Frame_width_rads / Frame_width_px); // gets radians per pixel conversion
    float center_coord_pixels = (Frame_width_px / 2.0f); // gets center of the frame in pixels along the x-dimension -- uses floating point division for better efficiency and no risk in variable type mismatch

    float pos_diff_pixels = target_x - center_coord_pixels ; // gets the difference in pixels (positive is to the right of the frame center and negative is to the left of the frame center) between an object and the center of the frame along the x-axis
    float rad_diff = pos_diff_pixels * rads_per_pixel; // convert to radians

    return rad_diff;

};




