/////////////////////////////////////////////////////////////
/*
Code Summary: This code takes in the 16 bit serial UART data that has been repackaged into 8 bit serial UART sections sent from the Jetson using the ArduinoSend code. 
This code sets up the baud rate (bits per second/bps) at a high speed/efficient data transfer option in the setup loop. During the main loop, the Arduino first reads
in sequentially each 8 bit package of data sent from the Jetson and assigns it to the variable that it is representing. After this, the binary data that was moved
and packaged into 8 bit packages when sent to the Arduino is repackaged together into 16 bit data representing the x and y coordinates of any given target as well 
as its object ID.  

This script specifically simulates/tests that the receiving end of the given Arduino Board is functioning (separate from any connection to the Jetson board) 
by creating and sending test targets to the Arduino in the same format that the Jetson would package the data into.

Author: Basic Framework by Gemini -- edited, modified and changed for the given application by Graeme Appel

Last Updated: 6/24/2026
*/

/////////////////////////////////////////////////////////////

// Preallocating necessary Variables

// Global declarations for target tracking states -- note that for the Arduino system unsigned integer types and integer types of specific bytes are used to prevent any mismatch betweeen coordinate dimensions sent form the Jetson to the arduino. 
// The "sending" function/side of things also uses these same data types to ensure functionality.
uint16_t activeTargetID = 0;
int16_t  targetXCoord   = 0;
int16_t  targetYCoord   = 0;

/////////////////////////////////////////////////////////////

void setup() {

    // Initialize Serial UART connection. Must perfectly match the B115200 
  // speed forced by tcsetattr() inside ArduinoSend::initializePort().
  Serial.begin(115200);
  
  // Optional: Pin initializations for system hardware (gimbals, servos, etc.)
  // pinMode(9, OUTPUT); 

  // Wait for serial monitoring to open and then output a message when setup is successful
  while(!Serial); // Wait for Serial Monitor to open
  Serial.println("Arduino Packet Processing Test Active");

}

/////////////////////////////////////////////////////////////

void loop() {

  /////////////////////////////////////////////////////////////
  // Simulating the SENDING part of the code -- with arbitrary values
  /////////////////////////////////////////////////////////////

  static unsigned long lastSend = 0; // preallocates a large variable that stores the amount of milliseconds since the last time a target/data package was sent

  if (millis() - lastSend > 3000) { // Fires every 3 seconds
    lastSend = millis();


  // Create random values for object ID as well as X and Y position to be sent over to the Arduino (all ranges of numbers are inside what is normal for each variable type but are otherwise arbitrarily decided)
  uint16_t test_object_ID = random(0, 60000);
  int16_t test_X_pos = random(-100, 100);
  int16_t test_Y_pos = random(-100, 100);

  // Print to the console which target IDs and positions are being sent at a given time
  Serial.print("\nInjecting Mock Target -> ID: "); Serial.print(test_object_ID); 
  Serial.print(" X: "); Serial.print(test_X_pos);
  Serial.print(" Y: "); Serial.println(test_Y_pos);

  // Call User Defined function (at bottom) to simulate the normal data packaging and sending that would be done on from the Jetson using ArduinoSend
  incomingDataFormatting (test_object_ID, test_X_pos, test_Y_pos); 

  delay(2); // Gives the hardware buffer 2ms to cleanly settle before the receiver
  }
  /////////////////////////////////////////////////////////////
  // Simulating the RECEIVING part of the code
  /////////////////////////////////////////////////////////////

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


      // Print verification to the monitor to prove reconstruction worked
        Serial.println("[SUCCESS] Packet parsed by Arduino!");
        Serial.print("Reconstructed ID: "); Serial.println(activeTargetID);
        Serial.print("Reconstructed X:  "); Serial.println(targetXCoord);
        Serial.print("Reconstructed Y:  "); Serial.println(targetYCoord);

        // Commented out for now -- only need if any of the PID control portion is worked on in here.
        // // Dispatch the synchronized coordinates straight into the tracking loop
        // executePIDControl(activeTargetID, targetXCoord, targetYCoord);
      }
      else {
        // Alignment error recovery: discard data frame to resynchronize stream
        // This prevents tracking drift if bytes get dropped mid-flight.
        while (Serial.available() > 0 && Serial.read() != '\n');
      }
    }
  }


}

/////////////////////////////////////////////////////////////
// User Defined Function(s)

void parseSimulatedPacket(uint8_t* testBuffer) {
  // Verify the start marker frame
  if (testBuffer[0] == '!') {
    uint8_t id_high = testBuffer[1];
    uint8_t id_low  = testBuffer[2];
    uint8_t x_high  = testBuffer[3];
    uint8_t x_low   = testBuffer[4];
    uint8_t y_high  = testBuffer[5];
    uint8_t y_low   = testBuffer[6];
    uint8_t end_marker = testBuffer[7];

    if (end_marker == '\n') {
      // Reconstruct the 16-bit units using your exact logic
      activeTargetID = (uint16_t)((id_high << 8) | id_low);
      targetXCoord   = (int16_t)((x_high << 8) | x_low);
      targetYCoord   = (int16_t)((y_high << 8) | y_low);

      // Print clean, human-readable confirmation text
      Serial.println("PACKET PROCESSING VERIFICATION");
      Serial.print("Successfully Reconstructed ID: "); Serial.println(activeTargetID);
      Serial.print("Successfully Reconstructed X:  "); Serial.println(targetXCoord);
      Serial.print("Successfully Reconstructed Y:  "); Serial.println(targetYCoord);
      Serial.println("\n\n");
    }
  }
}







// Here we have a helper function that simulates the same memory packaging that is seen in the ArduinoSend code meant for the Jetson in Linux

void incomingDataFormatting (uint16_t id, int16_t x, int16_t y)  {

  uint8_t buffer[8]; // unsigned 8 bit integer that splits up each 16 bit x and y coordinate into two parts to be able to be transmitted over the processors of both the Jetson and the Arduino

    buffer[0] = '!';                         // Start Frame Marker -- interacts with arduino portion of the code and Arduino only starts to process and store a piece 
                                             // of data if it sees this starting symbol (in binary) first. Prevents mistakes for if the arduino starts picking up a transmission
                                             // halfway through, then it knows where to start collecting a given set of x,y coordinate numbers

    // Split 16-bit ID into two 8-bit bytes
    buffer[1] = (id >> 8) & 0xFF;              // ID High Byte
    buffer[2] = id & 0xFF;                     // ID Low Byte


    buffer[3] = (x >> 8) & 0xFF;             // X High Byte (largest powers in binary to decimal) -- the x >> 8 shifts all the bits 8 to the left effectively making the higher bit into the lower bit temporarily and then the 0xFF
                                             //  is 1111 1111 in binary which when applied with the bitwise & operator, this only keeps bits with any 1's in them meaning the high bit which was moved to the lower bit is then kept 
                                             // and all the other information is discarded.
    buffer[4] = x & 0xFF;                    // X Low Byte -- same process as above is applied except no rightward shifting by 8 is applied so in this case, the only bit that is eliminated is the higher bit keeping the lower bit.
    buffer[5] = (y >> 8) & 0xFF;             // Y High Byte -- same process as above but for the input y coordinate
    buffer[6] = y & 0xFF;                    // Y Low Byte -- same process as above but for the input y coordinate
    buffer[7] = '\n';                        // End Frame Marker -- acts nearly identically to the start frame marker and tells the arduino to not accept a piece of binary data unless it is framed at the start by a "!" and ended with a "\n"

  // Write all newly packaged data to the Arduino. Note that normally this would be through the USB connection from the Jetson to the Arduino but in this case since we are simulating sending packages of data this will simply be to send
  // it from the main loop of this script to the Arduino to process.

  // for (int i = 0; i < 8; i++) { // loop through all indices of the repackaged x and y coordinates and object IDs as done above

  //   Serial.write(buffer[i]);

  parseSimulatedPacket(buffer);

  }






















