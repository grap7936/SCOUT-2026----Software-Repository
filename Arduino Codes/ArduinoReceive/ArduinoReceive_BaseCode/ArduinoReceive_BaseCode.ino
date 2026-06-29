/////////////////////////////////////////////////////////////
/*
Code Summary: 

TRACKING MODE: This code takes x and y coordinates as well as object IDs from a sendTargetCoordinates function in C++ meant for the linux environment in VNC viewer for the Jetson and RaspberryPI.
All of this data 16 bit serial UART data that has been repackaged into 8 bit serial UART sections sent from the Jetson using the ArduinoSend code. 
This code sets up the baud rate (bits per second/bps) at a high speed/efficient data transfer option in the setup loop. During the main loop, the Arduino first reads
in sequentially each 8 bit package of data sent from the Jetson and assigns it to the variable that it is representing. After this, the binary data that was moved
and packaged into 8 bit packages when sent to the Arduino is repackaged together into 16 bit data representing the x and y coordinates of any given target as well 
as its object ID.  

TESTING MODE: target x and y inputs are still taken in just as in testing mode in the same 16 to 8 then back to 16 bit converted format (object ID inputs are always 0 as they are not 
relevant for the testing side). Negative y coordinate inputs trigger each different test function detailed in the C++ script entitled "testArduinoMain.cpp" and also at the functions at 
the bottom of this code. Each function has a negative y input which indicates which test function should be called and then the x coordinate input is a command parameter also passed into
each function which helps to execute the required functionality, such as moving a servo motor a certain number of degrees.

Author: Graeme Appel

Last Updated: 6/29/2026
*/


/////////////////////////////////////////////////////////////

// Necessary Library Inclusions

#include <SimpleFOC.h> // library with applicable functions for the Brushless DC (BLDC) motor controlled by a SimpleFOC (Field Oriented Control)

/////////////////////////////////////////////////////////////

// Preallocating necessary Variables

// Global declarations for target tracking states -- note that for the Arduino system unsigned integer types and integer types of specific bytes are used to prevent any mismatch betweeen coordinate dimensions sent form the Jetson to the arduino. 
// The "sending" function/side of things also uses these same data types to ensure functionality.
uint16_t activeTargetID = 0;
int16_t  targetXCoord   = 0;
int16_t  targetYCoord   = 0;


float motor_target_angle = 0.0; // Continuous absolute angle tracking variable for the testMotor function and for keeping track of angle for tracking -- in [rads]
float motor_target_velocity = 0.0; // Continuous absolute velocity tracking variable for the Sentry mode and for keeping track of velocity for tracking -- in [rads/s]

// Initialize necessary motor setup details:
const int PPR = 2048; // Motor Resolution/PPR (Pulses Per Revolution) -- set as an arbitrary 2048 number for now before knowing accurate motor specs
const int Pole_Pairs = 11; // number of Pole Pairs the given motor has -- set arbitrarily for now

// BLDC motor instance matching the number of pole pairs the motor has as an input 
BLDCMotor motor = BLDCMotor(Pole_Pairs);

// 2. Driver instance specifying the 3 PWM pins and the EN pin shown in your diagram
BLDCDriver3PWM driver = BLDCDriver3PWM(11, 10, 9, 8);  // Inputs: 1.) IN Pin #1 connection on the Arduino to the simpleFOC board -- controls 1 pair of MOSFETs internally
                                                       //         2.) IN Pin #2 connection on the Arduino to the simpleFOC board -- controls 1 pair of MOSFETs internally
                                                       //         3.) IN Pin #3 connection on the Arduino to the simpleFOC board -- controls 1 pair of MOSFETs internally
                                                       //         4.) Enable Pin number that the simpleFOC board is attached to the Arduino -- acts as the master hardware safety switch for the motor driver board.

// 3. Encoder instance utilizing the interrupt pins 2 and 3
Encoder encoder = Encoder(2, 3, PPR); // Inputs: 1.) Digitial PWM/Interrupt pin plug in #1
                                      //         2.) Digitial PWM/Interrupt pin plug in #2
                                      //         3.) Motor PPR (Pulses Per Revolution) -- i.e basically the motors resolution

/////////////////////////////////////////////////////////////

// Define our system states
enum State { TEST_MODE, TRACKING_MODE };
State currentState = TEST_MODE; // Start in Test Mode -- note that we can trigger TRACKING_MODE when we are ready to receive target coordinates and IDs by inputting into the 

/////////////////////////////////////////////////////////////

// Forward Declare all functions for compiler
void executeTestCommands(int16_t command_parameter, int16_t command_call_ID);
void startSentryMode(int16_t command_parameter, int16_t command_call_ID);
void testMotor(int16_t command_parameter, int16_t command_call_ID);
void pingBilateralComms(int16_t command_parameter, int16_t command_call_ID);

 // ---------------------- Space for Mansi's test PID function here: -------------------------------------

// Forward declaration for interrupter functions included in the SimpleFOC library above
void doA();
void doB();

/////////////////////////////////////////////////////////////



void setup() {

  // Initialize Serial UART connection. Must perfectly match the B115200 
  // speed forced by tcsetattr() inside ArduinoSend::initializePort().
  Serial.begin(115200);
  
  // Attach the servo motor to a specific 

  // Initializing the hardware layers using built-in library functions
  encoder.init(); // sets up the embedded software to read electrical square waves and convert to dial inputs

  // Hardware-links the external physical changes on Digital Pins 2 and 3 directly to your user-defined background functions (doA and doB).
  attachInterrupt(digitalPinToInterrupt(2), doA, CHANGE);  // Hardware-links the external physical changes on Digital Pins 2 and 3 directly
                                                           // to your user-defined background functions (doA and doB). Standard code loops
                                                           // execute sequentially, which is too slow to catch high-speed motor signals. If
                                                           //  the motor spins and your Arduino is busy waiting for a serial data packet from the
                                                           //  Jetson, it will miss the encoder steps entirely. An interrupt instantly pauses the main 
                                                           // program loop for a fraction of a microsecond to record the exact movement step, ensuring 100% accurate position tracking.
  attachInterrupt(digitalPinToInterrupt(3), doB, CHANGE);

  driver.init(); // Configures pins 11, 10, and 9 as high-frequency PWM outputs and sets the hardware enable pin (Pin 8) to wake up the SimpleFOC Mini gate driver chip.
    
  // Linking the pieces together using built-in library functions
  motor.linkDriver(&driver); // Directly pairs your motor math object with the physical driver instance you configured on pins 11, 10, 9, and 8.
  motor.linkSensor(&encoder); // Hardware-links the external physical changes on Digital Pins 2 and 3 directly to your user-defined background functions (doA and doB).

  // Booting the FOC math engine
  motor.init(); // Allocates system memory for the tracking variables and loads your motor parameters (like the 11 pole pairs) into the controller.
  motor.initFOC(); // Triggers the physical startup calibration sequence.
                   // Note: The motor will make a short, audible humming sound or slightly nudge into position.

}

/////////////////////////////////////////////////////////////


void loop() {
  
  // the Arduino needs to read the motor encoder and have the motor check in with the encoder to ensure that whenever Arduino finishes a command/task it checks in with motor coils again
  motor.loopFOC();

  // DYNAMIC CONTROL BRANCH: Execute motor movement based on the active control engine state
  if (motor.controller == MotionControlType::velocity) {

    motor.move(motor_target_velocity); // Feeds target in radians per second [rads/s] for sentry testing and velocity tracking

  } 
  else {

    motor.move(motor_target_angle);    // Feeds target in absolute position radians [rads] for testMotor and angle tracking

  }

  // Check if at least one complete 8-byte packet is waiting in the buffer
  if (Serial.available() >= 8) {
    

    // Note that not every single variable inside of the buffer array (buffer[]) in the ArduinoSend function need to be accessed specifically
    //  but each byte is accessed sequentially by using the Serial read functions in order and so that is what assigns each byte split into 2 to different variable names in the ArduinoReceive function.
    // Peek or read the first byte to verify it's our designated frame marker
    if (Serial.read() == '!') {
      

      // Read the 16-bit Target ID (Split across 2 bytes)
      uint8_t id_high = Serial.read();
      uint8_t id_low  = Serial.read();
      
      // Read the 16-bit X pixel coordinate (Split across 2 bytes)
      uint8_t x_high  = Serial.read();
      uint8_t x_low   = Serial.read();
      
      // Read the 16-bit Y pixel coordinate (Split across 2 bytes)
      uint8_t y_high  = Serial.read();
      uint8_t y_low   = Serial.read();
      
      // Read the terminating character
      uint8_t end_marker = Serial.read();

      // Enforce frame validation alignment before trusting data integrity -- i.e only reads in and stores data pertaining to targets if the proper 
      // start and end marking conditions are met to ensure the packet of data is packaged properly.
      if (end_marker == '\n') {
        
        // Reconstruct the 16-bit words by shifting the High Byte up 8 bits 
        // and merging the Low Byte using a bitwise OR operation.

        // Recall that each high bit was shifted 8 bytes to the right to properly package each section in the ArduinoSend function and so this reverses that and moves the bit 8 bytes to the left 
        // Now, the "right" bit for each which for the the shifted high bit is all 0's and the right bit for the low bit is all 1's and 0's. The bitwise OR operation checks to see which "right" bit has members other than all 0's and takes that bit and combines it with the shifted high bit. 
        // Essentially, each of these lines stitches back together the split up bits and puts it back into a 16 byte unit for each case so for the ID this stores all of the target ID final numbers in binary and for the target coordinates this puts back toether the x and y coordinates. 
        activeTargetID = (uint16_t)((id_high << 8) | id_low);
        targetXCoord   = (int16_t)((x_high << 8) | x_low);
        targetYCoord   = (int16_t)((y_high << 8) | y_low);

      /////////////////////////////////////////////////////////////

        // STATE 1: TEST MODE -- Executes Tests by taking in inputs defined by UDF: executeTestCommands and blocks target tracking data until exiting test mode via user input

      /////////////////////////////////////////////////////////////

        
        if (currentState == TEST_MODE) {
          if (targetYCoord == -5) { // input -5 from the "Send" Jetson side to enable TRACKING_MODE to receive target position and ID input data
          
            currentState = TRACKING_MODE;

            // Ensure tracking mode defaults back to absolute angle control as all tracking and movement relies on moving based off of angle and not off of any constant velocity input
            motor.controller = MotionControlType::angle;
            motor_target_angle = 0.0;

            Serial.println(F("SYSTEM STATE UPDATE: All Desired Tests Verified. Switching to TRACKING_MODE."));
            Serial.println("To return to TESTING_MODE, input target_Y_coordinate = -6.");
            Serial.println("Now accepting live target ID and Position Data.\n");


          } 
          else if (targetYCoord < 0) { // Process individual test routines safely
            
            executeTestCommands(targetXCoord, targetYCoord);
          } 
          else { // Guard clause: Reject live coordinates sent prematurely
            
            Serial.println("[BLOCKED] Tracking packet ignored. System is still in TEST_MODE.");
          }
        }

        /////////////////////////////////////////////////////////////

        // STATE 2: LIVE TRACKING MODE

        /////////////////////////////////////////////////////////////

        else if (currentState == TRACKING_MODE) {

          if (targetYCoord == -6) {

            currentState = TEST_MODE;

            // Make all motor movement stop and revert the motor to its default angle to be able to begin testing again
            motor_target_velocity = 0.0;
            motor_target_angle = 0.0;

            Serial.println(F("SYSTEM STATE UPDATE: Returning to TEST_MODE."));
            Serial.println(F("Live tracking suspended. Run any desired tests.\n"));

          }

         else if (targetYCoord >= 0) {

          // Live target operations are all based on on absolute angle measurements -- this can be changed later if needed
          motor.controller = MotionControlType::angle;
            
          // Insert your active live camera tracking PID coordinates to motor_target_angle assignment here

          // Call motor and PID control functions


            Serial.println(F("PACKET PROCESSING VERIFICATION"));
            Serial.print(F("Successfully Reconstructed ID: ")); Serial.println(activeTargetID);
            Serial.print(F("Successfully Reconstructed X:  ")); Serial.println(targetXCoord);
            Serial.print(F("Successfully Reconstructed Y:  ")); Serial.println(targetYCoord);
            Serial.println("\n");

            
          } else {

            Serial.println(F("[WARNING] Test packet ignored. System locked in TRACKING_MODE."));

          }
        }
      } 
      else {
        // Stream resynchronization: Run if header marker was '!' but end marker was corrupt
        while (Serial.available() > 0 && Serial.read() != '\n');
      }
    }
    else {
      // Stream resynchronization: Run if the first byte extracted was not our designator '!'
      while (Serial.available() > 0 && Serial.read() != '\n');
    }
  }
}



/////////////////////////////////////////////////////////////
// User Defined Functions
/////////////////////////////////////////////////////////////

// Interrupt functions included in the SimpleFOC libary -- they must be initialized as user defined functions even though they are naturally included in the provided library
void doA() { // simulates the equivelent of the interrupt functions from the <Servo.h> by having channels to keep track of motor rotation

   encoder.handleA();

}

void doB() { // simulates the equivelent of the interrupt functions from the <Servo.h> by having channels to keep track of motor rotation

  encoder.handleB();

}

/* 1.) Sentry Mode Activation function

Function description:  Gives a constant rate of rotation to the motor connected to the Arduino to test that the movement without any camera and stream inputs is functional.

Inputs:

1.) int16_t command_parameter == In this case, this is an input velocity in rotations per minute (RPM) of the motor system. This is how fast the motor will be rotating in sentry mode.

2.) int16_t command_call_ID == redundant variable but called for completeness in this case. In the execute tests function this is called with specific negative number inputs to call each test function.


Outputs:
None


*/

void startSentryMode(int16_t command_parameter, int16_t command_call_ID) {

// In order to have a constant rate of motor spin, the motor needs to be in velocity control mode -- note that this will be switched to angle mode for the motor testing as we want to see specific controlled movements
motor.controller = MotionControlType::velocity;

Serial.println(F("COMMAND: Sentry Test Engaged"));
Serial.print(F("Motor will rotate at a constant rate until another test input is received"));
Serial.print(F("Target velocity received (RPM): ")); Serial.println(command_parameter);

// Dynamically switch SimpleFOC into constant velocity tracking mode
motor.controller = MotionControlType::velocity;

// Convert incoming RPM parameters to radians per second: rad/s = RPM * (2 * PI / 60)
motor_target_velocity = (float)command_parameter * (2.0 * PI / 60.0);

}



/* 2.) Motor activation function

Function description: Takes in a command parameter input by the x coordinate of the target position after this function is engaged with a -2 input for the 
command_call_ID (y coordinate for target). The command parameter input represents a degree input change for the motor and this is then converted to radians
and then read into a move motor function to move the motor a desired amount. Tests motor movement much more precisely than the sentry mode.

Inputs:

1.) int16_t command_parameter == The command parameter in this case is a degree measurement sent from the user to this system which is converted to radians and then the motor is moved that amount.

2.) int16_t command_call_ID == redundant variable but called for completeness in this case. In the execute tests function this is called with specific negative number inputs to call each test function.

Outputs:
None


*/

void testMotor(int16_t command_parameter, int16_t command_call_ID) {

// Switch to angle control mode for the motor so that specific controlled movements can be initiated
motor.controller = MotionControlType::angle;

Serial.println(F("COMMAND: Motor Test Mode engaged"));
Serial.print(F("Target angle received (degrees): ")); Serial.println(command_parameter);

// Convert input test angle (command_parameter) to radians to be used effectively in the motor.move function as inputs are required to be in redians
motor_target_angle = (float)command_parameter * (PI / 180.0);

// Moving the Simple FOC motor from its starting position by a given angle change given by the command parameter is done at the top of the main code

}




/////////////////////////////////////////////////////////////


/* 3.) Ping Function to test Bilateral comms

Function description: Ping mode -- sending test byte of data to ensure Arduino + PID system is connected to the Jetson.  Formats and intakes data the same way as
when processing normal targets but a negative input (-2) for y coordinate (command_call_ID) causes this function to be called to test functionality of the bilateral
USB connection between the Jetson and Arduino by sending a bit back to the console of the jetson in linux for the coder to recognize.

Inputs:

1.) int16_t command_parameter == a positive or negative number passed in (by the already recognized and processed data type of target x-coordinate above)
that will apply specifications to the test being run -- recall the motor example given above.

2.) int16_t command_call_ID == this will some given negative number ID passed in (by the already recognized and processed data type of target 
y-coordinate above) which depending on what negative number is passed in, it will use if and else if to trigger different data processing for different test commands.


Outputs:
None


*/


void pingBilateralComms(int16_t command_parameter, int16_t command_call_ID) { 

Serial.println(F("COMMAND: Bilateral Communications Connection testing engaged"));
Serial.print(F("Test Byte Value received: ")); Serial.println(command_parameter);

}


/////////////////////////////////////////////////////////////

/* 4.) PID controller function -- provided by Mansi

Function description:

Inputs:

Outputs:


*/

/////////////////////////////////////////////////////////////

/* 5.) Motor movement and tracking function -- code likely provided by Wyatt or to be figured out later

Function description:

Inputs:

Outputs:


*/


/////////////////////////////////////////////////////////////

/* 6.) Execute test commands function -- Parent function for all above functions

Function description: this contains all the conditional statements and negative values sent by different .cpp test scripts which trigger 
                      the specific tests such as testing camera image taking; bilateral comms connection; motor movement and connection e.t.c.

                      Note: The overall framework is that when negative Y coordinates are sent to the Arduino, this will trigger test commands because any actual 
                      data for targets sent to the Arduino will always be positive as (0,0) is situated in the upper left corner of any mage frame so all REAL targets
                      will have postiive x and ypositions and IDs. It is just convenient to use the already passed int16_t variables for target X and Y coordinates
                      also as inputs to trigger different test cases without having to define more variables.
                      

When X coordinates are passed in, this can be an input parameter that can help to apply specifications to the actual test being run. For example, 
for a motor test, this could tell the motor how many degrees to move on its dial to ensure functionality. The x-coordinate is applied here because the PID loop 
only takes an input of a sent x-coordinate because the PID can only cause motor sweep in 1 axi.

Inputs: 

1.) int16_t command_parameter == a positive or negative number passed in (by the already recognized and processed data type of target x-coordinate above)
that will apply specifications to the test being run -- recall the motor example given above.

2.) int16_t command_call_ID == this will some given negative number ID passed in (by the already recognized and processed data type of target 
y-coordinate above) which depending on what negative number is passed in, it will use if and else if to trigger different data processing for different test commands.


Outputs:
TBD


*/

/////////////////////////////////////////////////////////////

void executeTestCommands(int16_t command_parameter, int16_t command_call_ID) {

// Create If-else statement structure to receive inputs for specific test functionality of different components

if (command_call_ID == -1) { // Sentry mode of the gimbal-camera system -- this is the default mode to ensure movement before engaging tracking mode

Serial.println(F("COMMAND: Camera/Gimbal Sweep Mode Testing engaged"));
startSentryMode(command_parameter, command_call_ID); // Call Sentry Mode function

}

else if (command_call_ID == -2) { // Motor test mode -- after bilateral comms are checked, test motor by moving the motor to a certain postiion 

// Call Motor Testing Function
testMotor(command_parameter, command_call_ID);

}


else if (command_call_ID == -3) { // Ping mode -- sending test byte of data to ensure Arduino + PID system is connected to the Jetson

// Call Ping Function 
pingBilateralComms(command_parameter, command_call_ID);

}

else if (command_call_ID == -4) { // test PID control using Mansi's code provided later


}

else { // if any incorrect command_call_ID is entered then redirect to the provided command ID statements

Serial.println(F("Unrecognized Test Command Input received."));

}

}




