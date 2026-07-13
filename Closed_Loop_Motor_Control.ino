#include <SimpleFOC.h>

//  Hardware Pins    
const int ENCODER_PIN = 3;   // Hardware Interrupt pin for PWM encoder signal (White wire)

//  SimpleFOC Configuration 
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 10, 11, 8);
BLDCMotor motor = BLDCMotor(11); // 11 pole pairs 

// Defines PWM sensor on pin 3, reading raw pulse durations between 4us and 904us
MagneticSensorPWM sensor = MagneticSensorPWM(ENCODER_PIN, 4, 904);
// Fast hardware interrupt Service Routine (ISR) that triggers instantly on pin state changes
void doPWM(){ sensor.handlePWM(); }

//  Operational State Machine 
// TRACKING_CLOSED_LOOP
// SENTRY_OPEN_LOOP
enum SystemState { TRACKING_CLOSED_LOOP, SENTRY_OPEN_LOOP };
SystemState currentState = TRACKING_CLOSED_LOOP; // System defaults to tracking/holding position
bool stateChanged = false;                      

//  Motion Target Parameters 
float target_angle_radians = 0.0;       // Stores the closed-loop angle destination in radians
const float SENTRY_SPEED_RAD_S = 2.094;  // Sentry Spin Speed

void setup() {
  Serial.begin(115200);                  
  pinMode(LED_PIN, OUTPUT);                        

  // 1. Initialize Sensor Hardware
  sensor.init();                        
  sensor.enableInterrupt(doPWM);        
  motor.linkSensor(&sensor);        

  // 2. Initialize Driver Power Stage
  driver.voltage_power_supply = 12.0;    
  driver.init();                        
  motor.linkDriver(&driver);             
  
  // 3. Core Safety & Thermal Thresholds
  motor.voltage_limit = 3.0;            // Maximum phase voltage to 3V
  motor.velocity_limit = 30.0;          // Maximum motor velocity to 30 rad/s
  
  // 4. Closed-Loop Position Holding Tuning Parameters
  motor.PID_velocity.P = 0.1;           // Velocity P-gain: Lower values prevent rapid, laggy over-corrections
  motor.PID_velocity.I = 1.0;           // Velocity I-gain: Overcomes constant static friction while holding
  motor.PID_velocity.D = 0.001;         // Velocity D-gain: Kept ultra-low/zero to stop high-frequency buzzing
  motor.LPF_velocity.Tf = 0.03;         // Low Pass Filter (30ms): Blurs 1kHz sensor time-steps into smooth speed data
  motor.P_angle.P = 20.0;               // Angle P-gain: Dictates stiffness; higher means more aggressive holding torque
  motor.motion_downsample = 4;          // Run position calculations every 4 FOC loops to match 1kHz sensor rate

  // 5. Initial Boot Calibration Settings
  motor.voltage_sensor_align = 2.0;   
  
  // 6. Start Up System in Closed-Loop Mode
  motor.controller = MotionControlType::angle; // Force tracking mode initially
  Serial.println(F("Initializing Motor Hardware..."));
  motor.init();                      
  
  Serial.println(F("Performing Gentle Boot Alignment..."));
  motor.initFOC();                     
  
  target_angle_radians = motor.shaft_angle; // Set first target position to current natural resting angle
  Serial.println(F("System Online: Hybrid Control Active."));
}

void loop() {
  //  Core Motion Executive 
  if (currentState == TRACKING_CLOSED_LOOP) {
    motor.loopFOC();                    // Run high-speed space vector calculus loops for closed-loop fields
    motor.move(target_angle_radians);   
  } else {
    sensor.update();                    // Manually pull encoder hardware registers during open-loop sweep
    motor.move(SENTRY_SPEED_RAD_S);     // Drive magnetic field forward at a flat 20 RPM open-loop rate
  }

  //  State Transition Engine 
  if (stateChanged) {
    stateChanged = false;               // Reset transition flag to acknowledge processing
    
    if (currentState == SENTRY_OPEN_LOOP) {
      digitalWrite(LED_PIN
