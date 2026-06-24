/////////////////////////////////////////////////////////////
/*
Code Summary: This code takes in the 16 bit serial UART data that has been repackaged into 8 bit serial UART sections sent from the Jetson using the ArduinoSend code. 
This code sets up the baud rate (bits per second/bps) at a high speed/efficient data transfer option in the setup loop. During the main loop, the Arduino first reads
in sequentially each 8 bit package of data sent from the Jetson and assigns it to the variable that it is representing. After this, the binary data that was moved
and packaged into 8 bit packages when sent to the Arduino is repackaged together into 16 bit data representing the x and y coordinates of any given target as well 
as its object ID.  

Author: Basic Framework by Gemini -- edited, modified and changed for the given application by Graeme Appel

Last Updated: 6/24/2026
*/

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
