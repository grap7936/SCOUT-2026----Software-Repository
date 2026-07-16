/////////////////////////////////////////////////////////////

/*
Code Summary: This code sets up all necessary parameters for a simple FOC motor connected to an Arduino that is connected to a Jetson. After setting up the motor,
              driver and encoder with specified hardware parameters, the main loop receives 8 bit packages of data from the ArduinoSend and testArduinoMain scripts from the Jetson
              and runs this in a switch statement with different test cases. In the case of a -1 input sentry mode is engaged and the motor begins to spin at a constant rate which
              rotates the gimbal system. In the case of a -2 input, the motor is tested by moving it 1 full revolution (360 degrees) from its current position. In the case of a -3,
              a ping test is performed and sends a byte of data back from the Jetson to the Arduino then back to the Jetson to confirm the bilateral comms connection. In the case 
              of a -4 input that is the PID test. A -5 input switches from testing mode to tracking mode. A -6 input switches back from tracking to testing mode.

(Explanation from original main code for more detail on the Testing and tracking modes) -- a bit outdated but helps to give an idea of the design of testing and tracking mode
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

Last Updated: 7/16/2026
*/

/////////////////////////////////////////////////////////////

// Necessary Library Inclusions

#include "ReceiveEnd_Arduino.hpp" 
#include <SimpleFOC.h> // library with applicable functions for the Brushless DC (BLDC) motor controlled by a SimpleFOC (Field Oriented Control)
#include <PinChangeInterrupt.h> // library to change another pin on the NANO to be an interrupt pin -- we need to allow for 3 interrupt pins, 2 for tbe encoder, 1 for the camera

/////////////////////////////////////////////////////////////

// Global variable definitions
// volatile because these are written inside CamInterrupt() (an ISR) and read in loop().
// Without volatile the compiler may cache a stale copy and never see the ISR's updates.
volatile float CURRENT_MOTOR_POS = 0.0;
volatile int FRAME_NUM = 0; // Frame number counter starting from when the camera begins sending and processing data each time after an interrupt -- initialized to 0 for the time being, this will be changed later in the code.
extern float const Sentry_RPM = 10; // set the spinning motor speed to a default of 10 RPM. Likely this parameter won't be changed but if so the user input on the testingArduinoMain side can be altered

bool testModeActive = true;
float motor_target_angle = 0.0; // Continuous absolute angle tracking variable for the testMotor function and for keeping track of angle for tracking -- in [rads]
float motor_target_velocity = 0.0; // Continuous absolute velocity tracking variable for the Sentry mode and for keeping track of velocity for tracking -- in [rads/s]

// Initialize necessary motor setup details: (subject to change later once we know the specific motor parameters)
const int PPR = 2048; // Motor Resolution/PPR (Pulses Per Revolution) -- set as an arbitrary 2048 number for now before knowing accurate motor specs
const int Pole_Pairs = 11; // number of Pole Pairs the given motor has -- set arbitrarily for now
// Necessary camera parameters 
float const CAM_FRAME_WIDTH_PX = 2464.0f; // camera frame width in pixels
// float const CAM_FRAME_HEIGHT_PX = 2064.0f; // uncomment only if needed
// float const CAM_FRAME_HEIGHT_RAD = 6.09  * (PI/180.0f);// camera frame height in radians -- FOV converted from degree input
float const CAM_FRAME_WIDTH_RAD =  8.21 * (PI/180.0f); // camera frame width in radians -- FOV converted from degree input

// BLDC motor instance matching the number of pole pairs the motor has as an input 
BLDCMotor motor = BLDCMotor(Pole_Pairs);

// 2. Driver instance specifying the 3 PWM pins and the EN pin shown in your diagram
BLDCDriver3PWM driver = BLDCDriver3PWM(11, 10, 9, 8);  // Inputs: 1.) IN Pin #1 connection on the Arduino to the simpleFOC board -- controls 1 pair of MOSFETs internally
                                                       //         2.) IN Pin #2 connection on the Arduino to the simpleFOC board -- controls 1 pair of MOSFETs internally
                                                       //         3.) IN Pin #3 connection on the Arduino to the simpleFOC board -- controls 1 pair of MOSFETs internally
                                                       //         4.) Enable Pin number that the simpleFOC board is attached to the Arduino -- acts as the master hardware safety switch for the motor driver board.

// Encoder instance utilizing the interrupt pins 2 and 3
Encoder encoder = Encoder(2, 3, PPR); // Inputs: 1.) Digitial PWM/Interrupt pin plug in #1
                                      //         2.) Digitial PWM/Interrupt pin plug in #2
                                      //         3.) Motor PPR (Pulses Per Revolution) -- i.e basically the motors resolution

// Interrupt service routines required by SimpleFOC
void doA() { encoder.handleA(); }
void doB() { encoder.handleB(); }

// Updated from Pin 2 to Pin 4.
// Pins 2 and 3 are completely dedicated to high-speed encoder phase matching
// Pin 4 will now handle the camera's frame-sync pulse. -- this is done using the PinChangeInt library to reconfigure pin functionality
// Note depending on Electrical team's preferences, this might have to be changed to another pin but is set as pin 4 for the time being.

const int CAM_FRAME_PIN = 4;


// Camera / External Hardware Frame Interrupt
void CamInterrupt() {

    FRAME_NUM++; 
    CURRENT_MOTOR_POS = motor.shaft_angle;

}

/////////////////////////////////////////////////////////////

void setup() {

  // Current motor setup from ArduinoReceive_BaseCode is pasted in here for now, potentially the formatting of this may change later:

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

  // Attach the camera frame-sync interrupt so FRAME_NUM increments and CURRENT_MOTOR_POS
  // latches on each frame event. Without this, write() would always report Frame [0] Pos: 0.0000.
  // RISING assumes the camera pulses the line high once per frame -- change the edge (RISING/FALLING/
  // CHANGE) to match your camera's actual signal. See the CAM_FRAME_PIN note above re: pin choice.
  // Also: Replaced standard attachInterrupt logic.
  // Standard digitalPinToInterrupt() only works on pins 2 and 3. 
  // We swap it out for the library's pin-change macros to enable interrupts on Pin 4.
  pinMode(CAM_FRAME_PIN, INPUT);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(CAM_FRAME_PIN), CamInterrupt, RISING);

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
  
    // Keep the core FOC math loop running constantly
    motor.loopFOC();

    // Fetch serial packets from the background parsing layer
    JetsonPackage Data_Package = ArduinoReceive.read();

    // tracks whether this packet was the -2 command ID i.e the motor test. The -2 path prints its own bare
    // float telemetry (for the Jetson's std::stod) and must NOT be followed by write(), which would
    // append a labelled "SUCCESS:" string and corrupt the number the Jetson is trying to parse.
    bool sentMotorTelemetry = false;


    if (Data_Package.hasNewData) {
        
        // Handle Tracking Logic vs Testing System Logic via your command map
        if (!testModeActive && Data_Package.y >= 0) {
            
            // >= 0 -> target_position = PID(Command)
            motor.controller = MotionControlType::angle;

            // Compute the angular error between the target position and the center of the frame
            float rad_diff = ArduinoReceive.getRadDiff(CAM_FRAME_WIDTH_PX, CAM_FRAME_WIDTH_RAD, Data_Package.x);
            
            // -------------------------------------------------------------
            // Pass rad_diff to Mansi's PID code which then the result of that will be passed to the motor to move towards the target
            // ex: the line after the PID code would just be: motor_target_angle = motor.shaft_angle + rad_diff_after_PID; or just simply: motor_target_angle = MansiPID(rad_diff);
            // -------------------------------------------------------------
            
        } 

        // reject live (y >= 0) packets while still in test mode 
        else if (testModeActive && Data_Package.y >= 0) {
            Serial.println(F("[BLOCKED] Tracking packet ignored. System is still in TEST_MODE."));
        }

        else {
            // SAFETY FILTER: If we are in tracking mode, ignore any testing commands (y < 0) 
            // EXCEPT for state changes like -5 (enable tracking) or -6 (return to test) as well as -1 (starting sentry mode)
            if (!testModeActive && Data_Package.y != -5 && Data_Package.y != -6 && Data_Package.y != -1) {
                Serial.println(F("[BLOCKED] Testing command rejected. System is in TRACKING_MODE."));
            }

            else { 
                // Handle explicit command routines mapped via negative indicator states
                switch (Data_Package.y) {
                    case -1: // Spin at constant omega sentry sweep
                       ArduinoReceive.startSentryMode();
                        break;

                    case -2: // ArduinoReceive.test() - one full revolution & bare-float telemetry
                        ArduinoReceive.motor_test();
                        sentMotorTelemetry = true;   // suppress the write() below for this case
                        break;

                    case -3: // ArduinoReceive.ping() - send raw text validation verification 
                        ArduinoReceive.ping_bilateral_comms(Data_Package.x);
                        break;

                    case -4: // Reserved for alternative test branches (Mansi's PID code)
                        break;

                    case -5: // Disable testing bounds pipeline -> engage live target tracking
                        testModeActive = false;
                        motor.controller = MotionControlType::angle;
                        motor_target_angle = 0.0;
                        Serial.println(F("SYSTEM STATE UPDATE: Switching to TRACKING_MODE."));
                        break;

                    case -6: // Re-engage test modes and lock execution steps
                        testModeActive = true;
                        motor_target_velocity = 0.0;
                        motor_target_angle = 0.0;
                        Serial.println(F("SYSTEM STATE UPDATE: Returning to TEST_MODE."));
                        break;

                    default: // outputs feedback for unrecognized user inputs
                        Serial.println(F("Unrecognized Test Command Input received."));
                        break;
                }
            } 
        } 
    }

    // Pass target parameters to physical driver constraints
    if (motor.controller == MotionControlType::velocity) {
        motor.move(motor_target_velocity);
    } else {
        motor.move(motor_target_angle);
    }

    // Execute synchronous pipeline updates back to the Linux application host
    // Note: only send frame/position telemetry when we did NOT just emit the -2 bare float,
    // so the two response formats never collide in the serial buffer. 

    // I.e data packages of code are sent provided the -2 test in testing mode has been done first.

    // ENFORCE WRITE() ONLY DURING ACTIVE TRACKING
    // Only send the live camera frame and motor telemetry (write) if:
    // 1. We have new incoming data
    // 2. We did NOT just send test-telemetry (via case -2)
    // 3. Tracking mode is active (!testModeActive)
    // 4. It was actually a tracking coordinate packet (y >= 0)
    if (Data_Package.hasNewData && !sentMotorTelemetry && !testModeActive && Data_Package.y >= -1) {
        ArduinoReceive.write(FRAME_NUM, CURRENT_MOTOR_POS);
    }
}