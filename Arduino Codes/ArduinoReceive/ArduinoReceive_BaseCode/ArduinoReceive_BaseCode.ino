/////////////////////////////////////////////////////////////
/*
Code Summary: This code takes in the 16 bit serial UART data that has been repackaged into 8 bit serial UART sections sent from the Jetson using the ArduinoSend code. 
This code sets up the baud rate (bits per second/bps) at a high speed/efficient data transfer option in the setup loop. During the main loop, the Arduino first reads
in sequentially each 8 bit package of data sent from the Jetson and assigns it to the variable that it is representing. After this, the binary data that was moved
and packaged into 8 bit packages when sent to the Arduino is repackaged together into 16 bit data representing the x and y coordinates of any given target as well 
as its object ID.  

Author: Basic Framework by Gemini -- edited, modified and changed for the given application by Graeme Appel

Last Updated: 6/25/2026
*/

/////////////////////////////////////////////////////////////

// Preallocating necessary Variables

// Global declarations for target tracking states -- note that for the Arduino system unsigned integer types and integer types of specific bytes are used to prevent any mismatch betweeen coordinate dimensions sent form the Jetson to the arduino. 
// The "sending" function/side of things also uses these same data types to ensure functionality.
uint16_t activeTargetID = 0;
int16_t  targetXCoord   = 0;
int16_t  targetYCoord   = 0;

/////////////////////////////////////////////////////////////

// Define our system states
enum State { TEST_MODE, TRACKING_MODE };
State currentState = TEST_MODE; // Start in Test Mode -- note that we can trigger TRACKING_MODE when we are ready to receive target coordinates and IDs by inputting into the 

/////////////////////////////////////////////////////////////

void setup() {

    // Initialize Serial UART connection. Must perfectly match the B115200 
  // speed forced by tcsetattr() inside ArduinoSend::initializePort().
  Serial.begin(115200);
  
  // Optional: Pin initializations for system hardware (gimbals, servos, etc.)
  // pinMode(9, OUTPUT); 


}

void loop() {

  // Check if at least one complete 8-byte packet is waiting in the buffer
  if (Serial.available() >= 8) {
    

    // Note that not every single variable inside of the buffer array (bufffer[]) in the ArduinoSend function need to be accessed specifically
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

        Serial.println("Arduino is in TEST_MODE. Complete all desired tests and then input code to switch to TRACKING_MODE to process live target data.\n
                        Please enter:/n target_Y_coordinate = -1 for Camera/Gimbal Sweep mode test\n 
                                        target_Y_coordinate = -2 for Bilateral Communications Connection test\n
                                        target_Y_coordinate = -3 foor Motor mode Test\n
                                        target_Y_coordinate = -4 to move to TRACKING_MODE");

        if (currentState == TEST_MODE) {
          if (targetYCoord == -4) { // input -4 from the "Send" Jetson side to enable TRACKING_MODE to receive target position and ID input data
          
            currentState = TRACKING_MODE;
            Serial.println("SYSTEM STATE UPDATE: All Desired Tests Verified. Switching to TRACKING_MODE.");
            Serial.println("To return to TESTING_MODE, input target_Y_coordinate = -5.")
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

        else if (currentState == TRACKING_MODE) {

          if (targetYCoord == -5) {

            currentState = TEST_MODE;
            Serial.println("SYSTEM STATE UPDATE: Returning to TEST_MODE.");
            Serial.println("Live tracking suspended. Run any desired tests.\n");

          }

         else if (targetYCoord >= 0) {

            Serial.println("PACKET PROCESSING VERIFICATION");
            Serial.print("Successfully Reconstructed ID: "); Serial.println(activeTargetID);
            Serial.print("Successfully Reconstructed X:  "); Serial.println(targetXCoord);
            Serial.print("Successfully Reconstructed Y:  "); Serial.println(targetYCoord);
            Serial.println("\n");

            
          } else {

            Serial.println("[WARNING] Test packet ignored. System locked in TRACKING_MODE.");

          }
        }
      } // outer loop closing
      else {
        // Stream resynchronization
        while (Serial.available() > 0 && Serial.read() != '\n');
      }
    }
  }
}
        

// *************
// Send motor/mirror position information back to the Jetson here
// *************

// Might need time stamps for sending information back form the Arduino -- might have to set the arduino clock to the jetson clock -- look into if I just have to timestamp when the jetson receives information
// Look into if timestamping right when information is sent is necessary as opposed to time stamps for when the jetson processes it.

// Only the x-coordinate is sent to the PID controller as our gimbal-motor system can only move along one axis.


}

/////////////////////////////////////////////////////////////
// User Defined Functions
/////////////////////////////////////////////////////////////

/* 1.) Execute test commands function 

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

void executeTestCommands(int16_t command_parameter, int16_t command_call_ID) {

// Create If-else statement structure to receive inputs for specific test functionality of different components

if (command_call_ID == -1) { // Sweep mode of the gimbal-camera system -- this is the default mode to ensure movement before engaging tracking mode

Serial.println("COMMAND: Camera/Gimbal Sweep Mode Testing engaged");




}

else if (command_call_ID == -2) { // Ping mode -- sending test byte of data to ensure Arduino + PID system is connected to the Jetson

Serial.println("COMMAND: Bilateral Communications Connection testing engaged");
Serial.print("Test Byte Value received: "); Serial.println(command_parameter);







}

else if (command_call_ID == -3) { // Motor test mode -- after bilateral comms are checked, test motor by moving the motor to a certain postiion 

Serial.println("COMMAND: Motor Test Mode engaged")


}

else { // if any incorrect command_call_ID is entered then redirect to the provided command ID statements

Serial.println("Unrecognized Test Command Input received. Please enter:/n target_Y_coordinate = -1 for Camera/Gimbal Sweep mode test\n 
                                                                      target_Y_coordinate = -2 for Bilateral Communications Connection test\n
                                                                      target_Y_coordinate = -3 foor Motor mode Test\n
                                                                      target_Y_coordinate = -4 to move to TRACKING_MODE");

}



} 





