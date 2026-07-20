#include <math.h>
#include <SimpleFOC.h>

// --- Custom PID Data Structure ---
struct PID {
  float Kp, Ki, Kd, Kaw, T_C, T, max, min, max_rate;
  float integral, err_prev, deriv_prev, command_sat_prev, command_prev;
};

// --- Hardware Pins ---
const int ENCODER_PIN = 3;   // PWM encoder signal (White wire)
const uint8_t SLEEP_PIN = 7; // Sleep pin to control driver power state

// --- Control Settings ---
// Adjusted accuracy window to 0.015 rad (~0.85°) to prevent stiction grinding
const float ANGLE_THRESHOLD_RAD = 0.015; 
// Velocity threshold left exactly as it was original
const float VELOCITY_THRESHOLD_RAD_S = 0.1; 

// --- SimpleFOC Setup ---
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 10, 11, 8);
BLDCMotor motor = BLDCMotor(11); 
MagneticSensorPWM sensor = MagneticSensorPWM(ENCODER_PIN, 4, 904);
void doPWM(){ sensor.handlePWM(); }

// --- Global Control Variables ---
float target_angle_radians = 0.0;
unsigned long last_time = 0;

// --- Performance Tracking Variables ---
bool trackingActive = false;
unsigned long movementStartTime = 0;
float movementStartAngle = 0.0;

// Instantiate your custom PID struct
struct PID motor_pid;

// --- Custom PID Calculation ---
float PID_Step(struct PID *pid, float measurement, float setpoint) {
  float err = setpoint - measurement;
  pid->integral += pid->Ki * err * pid->T + pid->Kaw * (pid->command_sat_prev - pid->command_prev) * pid->T;
  float deriv_filt = (err - pid->err_prev + pid->T_C * pid->deriv_prev) / (pid->T + pid->T_C);

  pid->err_prev = err;
  pid->deriv_prev = deriv_filt;

  float command = pid->Kp * err + pid->integral + pid->Kd * deriv_filt;
  pid->command_prev = command;

  float command_sat = command;
  if (command > pid->max) command_sat = pid->max;
  else if (command < pid->min) command_sat = pid->min;

  if (command_sat > pid->command_sat_prev + pid->max_rate * pid->T)
    command_sat = pid->command_sat_prev + pid->max_rate * pid->T;
  else if (command_sat < pid->command_sat_prev - pid->max_rate * pid->T)
    command_sat = pid->command_sat_prev - pid->max_rate * pid->T;

  pid->command_sat_prev = command_sat;
  return command_sat;
}

void setup() {
  Serial.begin(115200);

  // 1. Configure Sleep Pin
  pinMode(SLEEP_PIN, OUTPUT);
  digitalWrite(SLEEP_PIN, HIGH); // Keep awake during initial calibration

  // 2. Initialize Sensor
  sensor.init();
  sensor.enableInterrupt(doPWM);
  motor.linkSensor(&sensor);

  // 3. Initialize Driver
  driver.voltage_power_supply = 12.0;
  driver.init();
  motor.linkDriver(&driver);

  // 4. Initialize Custom PID Parameters
  // { Kp, Ki, Kd, Kaw, T_C, T, max (volts), min (volts), max_rate (slew), structural zeros... }
  motor_pid = { 4.5, 1.2, 0.05, 0.3, 0.01, 0.001, 3.0, -3.0, 100.0, 0, 0, 0, 0, 0 };

  // 5. Configure SimpleFOC to receive raw Voltage/Torque commands from our PID
  motor.voltage_limit = 3.0; 
  motor.controller = MotionControlType::torque; 
  motor.torque_controller = TorqueControlType::voltage;

  // 6. Start Motor & Align Sensor
  Serial.println(F("Aligning Sensor..."));
  motor.init();
  motor.initFOC();

  target_angle_radians = motor.shaft_angle; 
  last_time = micros();
  
  Serial.println(F("System Online. Enter target angle in degrees:"));
}

void loop() {
  // --- 1. Real-Time dt (Sample Time) Calculation ---
  unsigned long now = micros();
  float dt = (now - last_time) / 1000000.0;
  last_time = now;

  // Protect against division-by-zero or serial parsing timing spikes
  if (dt <= 0.0 || dt > 0.1) {
    dt = 0.001; 
  }
  motor_pid.T = dt; 

  // --- 2. SimpleFOC Sensor & Math Core ---
  sensor.update();  
  motor.loopFOC();  

  // --- 3. Run Your Custom PID Loop & Dynamic Power Management ---
  float angle_error = abs(target_angle_radians - motor.shaft_angle);
  float current_velocity = abs(sensor.getVelocity()); 

  // WAKE UP if we are outside the target window OR if the motor is still actively moving
  if (angle_error > ANGLE_THRESHOLD_RAD || current_velocity > VELOCITY_THRESHOLD_RAD_S) {
    // MOTOR SHOULD MOVE: Wake up driver and apply custom PID calculations
    digitalWrite(SLEEP_PIN, HIGH); 
    
    float control_voltage = PID_Step(&motor_pid, motor.shaft_angle, target_angle_radians);
    motor.move(control_voltage);
  } 
  else {
    // MOTOR IS AT TARGET AND STOPPED: Sleep safely
    digitalWrite(SLEEP_PIN, LOW); 
    motor.move(0); 
    
    // Check if we just arrived from an active tracking target command
    if (trackingActive) {
      unsigned long arrivalTime = micros();
      float timeElapsedSeconds = (arrivalTime - movementStartTime) / 1000000.0;
      
      Serial.println(F("\n--- Target Reached ---"));
      Serial.print(F("Arrived at: ")); 
      Serial.print(motor.shaft_angle * (180.0 / PI)); Serial.println(F("°"));
      
      Serial.print(F("Starting Position: ")); 
      Serial.print(movementStartAngle); Serial.println(F("°"));
      
      Serial.print(F("Time: ")); 
      Serial.print(timeElapsedSeconds, 4); Serial.println(F(" seconds"));
      Serial.println(F("----------------------"));
      
      trackingActive = false; 
    }
    
    // Completely clear integral history to kill windup instantly
    motor_pid.integral = 0;
    motor_pid.command_prev = 0;
    motor_pid.command_sat_prev = 0;
    motor_pid.err_prev = 0;
    motor_pid.deriv_prev = 0;
  }

  // --- 4. Robust Serial Command Parser ---
  if (Serial.available() > 0) {
    char peekChar = Serial.peek();
    
    // Check if the incoming byte is a digit, minus sign, or decimal point
    if ((peekChar >= '0' && peekChar <= '9') || peekChar == '-' || peekChar == '.') {
      
      // Grab quiet absolute position data before triggering target adjustments
      sensor.update();
      movementStartAngle = sensor.getAngle() * (180.0 / PI); 
      
      float target_degrees = Serial.parseFloat();
      target_angle_radians = target_degrees * (PI / 180.0);
      
      movementStartTime = micros();
      trackingActive = true; 
      
      Serial.print(F("\nNew Target set to: "));
      Serial.print(target_degrees);
      Serial.println(F("°"));
    } else {
      // Strip out whitespace/newlines safely
      Serial.read(); 
    }
  }
}
