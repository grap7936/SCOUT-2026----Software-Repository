#include <SimpleFOC.h>

// Hardware Pins
const int LED_PIN = 12; // High = Sentry Mode active, Low = Tracking Mode active

// SimpleFOC Setup
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 10, 11, 8);
BLDCMotor motor = BLDCMotor(11); // Initialize BLDC motor with 11 pole pairs

// Operational States 
enum SystemState { TRACKING_MODE, SENTRY_MODE };
SystemState currentState = TRACKING_MODE; 
bool stateChanged = false; // Flag to trigger one-time prints and LED changes on transition

// Motion Profile Variables 
float target_angle_radians = 0.0;       // Stores the manual setpoint for Tracking Mode
const float SENTRY_SPEED_RAD_S = 2.094;  // Target rotational velocity for Sentry Mode (20 RPM)
float sentry_angle_accumulator = 0.0;    // Tracks the continuous target position during rotation
unsigned long last_update_time = 0;      // Stores timestamp of the previous loop run for dt calculation

void setup() {
  Serial.begin(115200);
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); 

  // Initialize Driver Power Stage
  driver.voltage_power_supply = 12.0;
  driver.init();
  motor.linkDriver(&driver);
  
  // Set Safety Limits & Open-Loop Control Mode
  motor.voltage_limit = 3.5;
  motor.velocity_limit = 30.0; 
  motor.controller = MotionControlType::angle_openloop;
  motor.init();
  
  last_update_time = micros();
  
  Serial.println("System Online: TRACKING MODE.");
  Serial.println("Enter angle in degrees, or type 's' to toggle SENTRY MODE.");
}

void loop() {
  unsigned long currentTimeMicros = micros();

  if (stateChanged) {
    stateChanged = false; 
    
    if (currentState == SENTRY_MODE) {
      digitalWrite(LED_PIN, HIGH); 
      sentry_angle_accumulator = motor.shaft_angle; // Lock onto current spot before starting rotation
      Serial.println("\n>>> SENTRY MODE ACTIVE (20 RPM) <<<");
    } else {
      digitalWrite(LED_PIN, LOW); 
      target_angle_radians = motor.shaft_angle; // Lock position right where the motor stopped spinning
      Serial.println("\n>>> TRACKING MODE ACTIVE <<<");
    }
  }

  // Serial Interface
  if (Serial.available() > 0) {
    char inputChar = Serial.peek(); // Look at first byte in buffer

    // Toggle mode if 's' or 'S' is received
    if (inputChar == 's' || inputChar == 'S') {
      Serial.read(); // Drop the mode character from the buffer
      currentState = (currentState == TRACKING_MODE) ? SENTRY_MODE : TRACKING_MODE;
      stateChanged = true;
    } 
    // Parse target angle if inside Tracking Mode
    else {
      if (currentState == TRACKING_MODE) {
        float degrees = Serial.parseFloat();
        target_angle_radians = degrees * (PI / 180.0); // Convert user input degrees to standard radians
        Serial.print("Target: "); Serial.print(degrees); Serial.println(" deg.");
      } else {
        Serial.read(); // Discard incoming numerical data if typing while in Sentry Mode
      }
    }
  }

  // Motion Execution
  if (currentState == TRACKING_MODE) {
    motor.move(target_angle_radians); // Hold steady at user-defined setpoint
  } else {
    // Sentry Mode: Calculate elapsed time delta (dt) in seconds
    float dt = (currentTimeMicros - last_update_time) / 1000000.0;
    
    // Safety check: Only move if the time gap is valid
    if (dt > 0.0 && dt < 0.1) {
      sentry_angle_accumulator += SENTRY_SPEED_RAD_S * dt;
    }
    motor.move(sentry_angle_accumulator); // Command continuous step updates
  }

  last_update_time = currentTimeMicros; // Store time for next calculation
}
