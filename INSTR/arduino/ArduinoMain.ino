/////////////////////////////////////////////////////////////

/*
Code Summary: 

Author: Graeme Appel

Last Updated: 7/8/2026
*/


/////////////////////////////////////////////////////////////

// Necessary Library Inclusions

#include "ReceiveEnd_Arduino.hpp" 
#include <SimpleFOC.h> // library with applicable functions for the Brushless DC (BLDC) motor controlled by a SimpleFOC (Field Oriented Control)

/////////////////////////////////////////////////////////////

// Global variable definitions
// volatile because these are written inside CamInterrupt() (an ISR) and read in loop().
// Without volatile the compiler may cache a stale copy and never see the ISR's updates.
volatile float CURRENT_MOTOR_POS = 0.0;
volatile int FRAME_NUM = 0; // Frame number counter starting from when the camera begins sending and processing data each time after an interrupt -- initialized to 0 for the time being, this will be changed later in the code.
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

// Pin the camera/external frame-sync signal is wired to. NOTE: On the ATmega328P (UNO/NANO) only pins 2 and 3 support attachInterrupt(), and both are
// already used by the encoder above. CAM_FRAME_PIN below is therefore a PLACEHOLDER -- set it to whatever pin your camera strobe actually lands on. If that pin is NOT 2 or 3, attachInterrupt()
// will not fire on it and you'll need a Pin Change Interrupt (e.g. the PinChangeInterrupt library) instead. This attaches cleanly as-is only if CAM_FRAME_PIN is a hardware-interrupt-capable pin.

const int CAM_FRAME_PIN = 2; // PLACEHOLDER -- change to the real camera frame-sync pin



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
  pinMode(CAM_FRAME_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(CAM_FRAME_PIN), CamInterrupt, RISING);

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
            
            // -------------------------------------------------------------
            // Pass target coordinates to Mansi's PID loop function here
            // motor_target_angle = MansiPID(Data_Package.x); 
            // -------------------------------------------------------------
            
        } 

        // reject live (y >= 0) packets while still in test mode 
        else if (testModeActive && Data_Package.y >= 0) {
            Serial.println(F("[BLOCKED] Tracking packet ignored. System is still in TEST_MODE."));
        }
        
        else {
            // Handle explicit command routines mapped via negative indicator states
            switch (Data_Package.y) {
                case -1: // Spin at constant omega sentry sweep
                    motor.controller = MotionControlType::velocity;
                    motor_target_velocity = Sentry_RPM * (2.0 * PI / 60.0);
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

    // Pass target parameters to physical driver constraints
    if (motor.controller == MotionControlType::velocity) {
        motor.move(motor_target_velocity);
    } else {
        motor.move(motor_target_angle);
    }

    // Execute synchronous pipeline updates back to the Linux application host
    // Note: only send frame/position telemetry when we did NOT just emit the -2 bare float,
    // so the two response formats never collide in the serial buffer.
    if (Data_Package.hasNewData && !sentMotorTelemetry) {
        ArduinoReceive.write(FRAME_NUM, CURRENT_MOTOR_POS);
    }
}
