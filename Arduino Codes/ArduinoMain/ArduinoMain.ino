/////////////////////////////////////////////////////////////

/*
Code Summary: 

Author: Graeme Appel, Wyatt Tran, Mansi Pahade

Last Updated: 7/7/2026
*/


/////////////////////////////////////////////////////////////

// Necessary Library Inclusions

#include <ReceiveEnd_Arduino.hpp> 
#include <SimpleFOC.h> // library with applicable functions for the Brushless DC (BLDC) motor controlled by a SimpleFOC (Field Oriented Control)

/////////////////////////////////////////////////////////////

// Global variable definitions
float CURRENT_MOTOR_POS = 0.0;
int FRAME_NUM = 0; // Frame number counter starting from when the camera begins sending and processing data each time after an interrupt -- initialized to 0 for the time being, this will be changed later in the code.
float const Sentry_RPM = 10; // set the spinning motor speed to a default of 10 RPM. Likely this parameter won't be changed but if so the user input on the testingArduinoMain side can be altered

bool testModeActive = true;
float motor_target_angle = 0.0; // Continuous absolute angle tracking variable for the testMotor function and for keeping track of angle for tracking -- in [rads]
float motor_target_velocity = 0.0; // Continuous absolute velocity tracking variable for the Sentry mode and for keeping track of velocity for tracking -- in [rads/s]

// Initialize necessary motor setup details: (subject to change later once we know the specific motor parameters)
const int PPR = 2048; // Motor Resolution/PPR (Pulses Per Revolution) -- set as an arbitrary 2048 number for now before knowing accurate motor specs
const int Pole_Pairs = 11; // number of Pole Pairs the given motor has -- set arbitrarily for now

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
// ***************************** Start by Reviewing this tomorrow morning and debugging *****************************************

void loop() {
  
    // Keep the core FOC math loop running constantly
    motor.loopFOC();

    // Fetch serial packets from the background parsing layer
    JetsonCommand cmd = ArduinoReceive.read();

    if (cmd.hasNewData) {
        
        // Handle Tracking Logic vs Testing System Logic via your command map
        if (!testModeActive && cmd.y >= 0) {
            
            // >= 0 -> target_position = PID(Command)
            motor.controller = MotionControlType::angle;
            
            // -------------------------------------------------------------
            // Pass target coordinates to Mansi's PID loop function here
            // motor_target_angle = MansiPID(cmd.x); 
            // -------------------------------------------------------------
            
        } 
        else {
            // Handle explicit command routines mapped via negative indicator states
            switch (cmd.y) {
                case -1: // Spin at constant omega sentry sweep
                    motor.controller = MotionControlType::velocity;
                    motor_target_velocity = SENTRY_RPM * (2.0 * PI / 60.0);
                    break;

                case -2: // ArduinoReceive.test() - one full revolution & telemetry
                    ArduinoReceive.test();
                    break;

                case -3: // ArduinoReceive.ping() - send raw text validation verification 
                    ArduinoReceive.ping(cmd.x);
                    break;

                case -4: // Reserved for alternative test branches
                    break;

                case -5: // Disable testing bounds pipeline -> engage live target tracking
                    testModeActive = false;
                    motor.controller = MotionControlType::angle;
                    motor_target_angle = 0.0;
                    break;

                case -6: // Re-engage test modes and lock execution steps
                    testModeActive = true;
                    motor_target_velocity = 0.0;
                    motor_target_angle = 0.0;
                    break;
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
    if (cmd.hasNewData) {
        ArduinoReceive.write(FRAME_NUM, CURRENT_MOTOR_POS);
    }
}














}
