#include <SimpleFOC.h>

// --- Hardware Pins ---
const int LED_PIN = 12;      // LED pin: HIGH in Sentry mode, LOW in Tracking mode
const int ENCODER_PIN = 3;   // Hardware Interrupt pin for PWM encoder signal (White wire)

// --- SimpleFOC Core Configuration ---
// Configures driver for 3 PWM signals on pins 9, 10, 11 with Enable line on pin 8
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 10, 11, 8);
// Defines motor with exactly 11 pole pairs (matching SparkFun ROB-27478)
BLDCMotor motor = BLDCMotor(11); 

// --- Encoder Interrupt Callback Setup ---
// Defines PWM sensor on pin 3, reading raw pulse durations between 4us and 904us
MagneticSensorPWM sensor = MagneticSensorPWM(ENCODER_PIN, 4, 904);
// Fast hardware interrupt Service Routine (ISR) that triggers instantly on pin state changes
void doPWM(){ sensor.handlePWM(); }

// --- Operational State Machine ---
// TRACKING_CLOSED_LOOP = Rigidly hold position using sensor feedback
// SENTRY_OPEN_LOOP     = Buttery-smooth constant speed without sensor jitter
enum SystemState { TRACKING_CLOSED_LOOP, SENTRY_OPEN_LOOP };
SystemState currentState = TRACKING_CLOSED_LOOP; // System defaults to tracking/holding position
bool stateChanged = false;                       // State change flag used to manage transitions

// --- Motion Target Parameters ---
float target_angle_radians = 0.0;       // Stores the closed-loop angle destination in radians
const float SENTRY_SPEED_RAD_S = 2.094;  // Target rotational speed for sentry sweep (20 RPM in rad/s)

void setup() {
  Serial.begin(115200);                  // Initialize serial interface at 115200 baud
  pinMode(LED_PIN, OUTPUT);              // Configure indicator pin as an output
  digitalWrite(LED_PIN, LOW);            // Initialize LED to off (Tracking mode indicator)

  // 1. Initialize Sensor Hardware
  sensor.init();                         // Wake up the magnetic sensor logic
  sensor.enableInterrupt(doPWM);         // Attach the fast ISR to pin 3 to handle pulse timings
  motor.linkSensor(&sensor);             // Link the hardware sensor instance to the motor object

  // 2. Initialize Driver Power Stage
  driver.voltage_power_supply = 12.0;    // Define the primary input supply voltage
  driver.init();                         // Initialize driver PWM timers and pin states
  motor.linkDriver(&driver);             // Link the hardware driver instance to the motor object
  
  // 3. Core Safety & Thermal Thresholds
  motor.voltage_limit = 3.0;            // Limits maximum phase voltage to 3V to eliminate motor overheating
  motor.velocity_limit = 30.0;          // Limits maximum motor velocity to 30 rad/s
  
  // 4. Closed-Loop Position Holding Tuning Parameters
  motor.PID_velocity.P = 0.05;          // Velocity P-gain: Lower values prevent rapid, laggy over-corrections
  motor.PID_velocity.I = 1.0;           // Velocity I-gain: Overcomes constant static friction while holding
  motor.PID_velocity.D = 0.001;         // Velocity D-gain: Kept ultra-low/zero to stop high-frequency buzzing
  motor.LPF_velocity.Tf = 0.05;         // Low Pass Filter (50ms): Blurs 1kHz sensor time-steps into smooth speed data
  motor.P_angle.P = 15.0;               // Angle P-gain: Dictates stiffness; lowered slightly to minimize holding jitter
  motor.motion_downsample = 4;          // Run position calculations every 4 FOC loops to match 1kHz sensor rate

  // 5. Initial Boot Calibration Settings
  motor.voltage_sensor_align = 2.5;     // Clean alignment voltage to help it pass initialization smoothly
  
  // 6. Start Up System in Closed-Loop Mode
  motor.controller = MotionControlType::angle; // Force tracking mode initially
  Serial.println(F("Initializing Motor Hardware..."));
  motor.init();                         // Initialize internal motor data structures
  
  // --- THE FIX: MANUAL PARAMETER HARDCODING ---
  // UNCOMMENT the two lines below once you read your values from the Serial monitor on a semi-successful run.
  // When these are set, motor.initFOC() completely skips the physical startup movement.
  // motor.zero_electric_angle = 1.57;    // Put your specific zero_electric_angle value here
  // motor.sensor_direction = Direction::CW; // Put your specific direction here (CW or CCW)
  
  Serial.println(F("Performing Boot Alignment..."));
  motor.initFOC();                      // Align sensor electrical angle to magnetic poles (or loads manual params)
  
  target_angle_radians = motor.shaft_angle; // Set first target position to current natural resting angle
  Serial.println(F("System Online: Hybrid Control Active."));
}

void loop() {
  // --- Core Motion Executive ---
  if (currentState == TRACKING_CLOSED_LOOP) {
    motor.loopFOC();                    // Run high-speed space vector calculus loops for closed-loop fields
    motor.move(target_angle_radians);   // Drive phases to maintain target angle with high holding torque
  } else {
    sensor.update();                    // Manually pull encoder hardware registers during open-loop sweep
    motor.move(SENTRY_SPEED_RAD_S);     // Drive magnetic field forward at a flat 20 RPM open-loop rate
  }

  // --- State Transition Engine ---
  if (stateChanged) {
    stateChanged = false;               // Reset transition flag to acknowledge processing
    
    if (currentState == SENTRY_OPEN_LOOP) {
      digitalWrite(LED_PIN, HIGH);      // Light up indicator LED to signal entry to Sentry mode
      motor.controller = MotionControlType::velocity_openloop; // Switch SimpleFOC engine to open-loop mode
    } 
    else {
      digitalWrite(LED_PIN, LOW);       // Turn off indicator LED to signal entry to Tracking mode
      
      // Sync positions instantly before entering closed loop to prevent violent snapping
      sensor.update();                  // Pull the absolute freshest position data from the encoder
      motor.shaft_angle = sensor.getAngle(); // Force sync SimpleFOC internal tracking variables to real position
      target_angle_radians = motor.shaft_angle; // Freeze holding target exactly where open-loop sweep stopped
      
      motor.controller = MotionControlType::angle; // Engage rigid closed-loop position control lock
    }
  }

  // --- Serial Interface Parser ---
  if (Serial.available() > 0) {
    char inputChar = Serial.peek();     // Inspect first byte in buffer without modifying it
    
    // Toggle state if character is 's' or 'S'
    if (inputChar == 's' || inputChar == 'S') {
      Serial.read();                    // Strip the processed state character from the buffer
      currentState = (currentState == TRACKING_CLOSED_LOOP) ? SENTRY_OPEN_LOOP : TRACKING_CLOSED_LOOP;
      stateChanged = true;              // Flag the state change for execution on next iteration
    } 
    // Handle angle targets if input is a digit or negative sign
    else if ((inputChar >= '0' && inputChar <= '9') || inputChar == '-') {
      float degrees = Serial.parseFloat(); // Non-blocking extraction of decimal angle string
      if (currentState == TRACKING_CLOSED_LOOP) {
        target_angle_radians = degrees * (PI / 180.0); // Convert user input degrees directly to radians
      }
    } else {
      Serial.read();                    // Clear out newlines or unhandled junk characters to keep buffer clear
    }
  }
}
