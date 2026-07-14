/////////////////////////////////////////////////////////////
/*

NOTE: Most lines of text (i.e std::cout) are to be used for debugging purposes only as if all of them are included in video processing the system will be very very very slow
Uncomment lines as necessary to debug, but the default should leave them all commented out for optimized performance. 

Data is sent in the format: ID, x, y for if needing to view numbers for each target without having any std::cout outputs for clarification.

Code Summary:  This code details the Jetson side of the Jetson to Arduino serial UART connection. This code has many functions. InitializePort() works to set up the serial connection between the Jetson and Arduino as well as enabling it to be seen by the user in realVNC viewer.
               sendTargetCoordinates() is the main function that sends the x and y coordinates of the detected debris to the Arduino for processing. It also has a built in read function that reads the telemetry data sent back from the Arduino to the Jetson.
               This is used to verify that the Arduino is receiving and processing the data correctly. readMotorPosition() receives the motor angle position from the ArduinoReceive end and takes in this information to later be stored inside of a text file using the logTelemetry fucnction.
               logTelemetry stores all desired data inside a text file which will be created using the main script of all of these codes and will likely be operated using the driver.sh master script.


Author: Graeme Appel

Last Updated: 7/14/2026
*/

/////////////////////////////////////////////////////////////

#include "ArduinoSend.hpp" // includes header file
#include <iostream> // standard C++ library
#include <cstring> // allows for processing of strings and character blocks
#include <fstream> // Included to write outputs to a .txt file
#include <cstdio> // Included <cstdio> to use sscanf for splitting the string into separate variables

// Included libraries for working in Linux code with the Jetson
#ifndef _WIN32
#include <fcntl.h> // file control library -- linux treats everything as files so this provides tools for opening different files and accessing and manipulating necessary file information
#include <termios.h> // this enables all terminal and I/O interfaces which allows for manual control over setting and looking at parameters and connection ports
#include <unistd.h> // helps link together the linux and C++ interfaces and supplies the functions like write() and close()
#endif

/////////////////////////////////////////////////////////////




ArduinoSend::ArduinoSend(const std::string& port) : serial_fd(-1), port_name(port), tracking_mode_active(false) {} // Initialized tracking mode to false so that it starts in testing mode

ArduinoSend::~ArduinoSend() {
    closePort();
}


/////////////////////////////////////////////////////////////

/* 1.) initializePort() function

Function description: This function initializes the serial port connection between the Jetson and Arduino. It sets the baud rate, data format, and other parameters necessary for communication.
 All of the specific parameters and specifications for setting up the system in linux are also here and also viewing inside of realVNC viewer or similar softwares to this is also defined and established here.

Inputs: 
None

Outputs:
1.) bool true/false == This is simply an arbitrary true or false variable that is used to check if the port was successfully initialized or not. If it was, it returns true and if it was not, it returns false.

*/


/////////////////////////////////////////////////////////////

bool ArduinoSend::initializePort() {
    // Open read/write, block execution on read, prevent it from becoming a controlling terminal
    serial_fd = open(port_name.c_str(), O_RDWR | O_NOCTTY);
    if (serial_fd == -1) {
        std::cerr << "[ERROR] ArduinoSend: Failed to open port " << port_name << std::endl;
        return false;
    }

    struct termios options;
    tcgetattr(serial_fd, &options);

    // Set standard Baud Rate to 115200 (Matches Arduino Serial.begin)
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    // Enforce 8N1 Data layout
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    // Turn off Hardware/Software Flow Control completely
    options.c_cflag &= ~CRTSCTS;
    options.c_iflag &= ~(IXON | IXOFF | IXANY);

    // Force Raw Processing Mode (Crucial for streaming pure binary integers safely)
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;

    options.c_cc[VMIN]  = 0;  // Non-blocking read: return immediately if no data
    options.c_cc[VTIME] = 1;  // Wait up to 1 decisecond (100ms) for incoming bytes

    // Apply configuration changes immediately
    tcsetattr(serial_fd, TCSANOW, &options);
    return true;
}

/////////////////////////////////////////////////////////////

/* 2.) sendTargetCoordinates() function

Function description:  This function details the data packaging and processing related to how data packets are sent from the Jetson to the Arduino while in either Tracking mode where real data is being sent from the camera's video stream, or in testing mode where data is sent in the same format but only by specific user input.
                       The Jetson and Arduino can only send 8 bit data across the serial UART connection and so the necessary 16 bit data is split into 2 offset binary halves of 8 bits for each section of data and sent over the serial UART connection this way and is tehn reassembled back into the 16 bit data on the ArduinoReceive
                       side to get back the original numeric data sent over for x and y postion and object ID.   The specification for whether the system is in tracking or testing mode only affects the delay output between the Arduino and Jetson. 
                     For testing mode, more delay time is needed for user input and set up parameters for the Arduino. For tracking mode, a much smaller delay optimizes speed of processing and is thus utilized.

Inputs: 
1.) uint16_t id == unsigned 16 bit integer used to store the given object ID of detected debris in a given frame of the camera's video stream. This is used to track the same piece of debris across multiple frames and is used to send the correct x and y coordinates of the debris to the Arduino for processing. 
This is specifcally an unsigned 16 bit variable because unsigned allows for storing more object IDs if in a very object dense area and negative object IDs for this code system do not exist. 

2.) int16_t x == signed 16 bit integer used to store the given x coordinate of detected debris in a given frame of the camera's video stream. This is used to send the correct x coordinate of a given detected debris to the Arduino for processing.
3.) int16_t y == signed 16 bit integer used to store the given y coordinate of detected debris in a given frame of the camera's video stream. This is used to send the correct y coordinate oof a given detected debris to the Arduino for processing.
4.) std::ofstream& logFile == This is a reference to an output file stream object that is used to log telemetry data to a text file. It allows the function to write information about the sent coordinates and the current state of the system (tracking or testing mode) to a log text file.

Outputs:
1.) bool true/false ==  returns a bool variable value to the system to ensure that the size of the written data is equal to the size of the stored process data. If they are not the same this means that there is likely some sort of wire disconnected which stops running the system allowing it to be fixed/debugged.

*/


/////////////////////////////////////////////////////////////

bool ArduinoSend::sendTargetCoordinates(uint16_t id, int16_t x, int16_t y, std::ofstream& logFile) {
    if (serial_fd == -1) return false; // exits the sending coordinates function early if no USB connection descriptor number is found preventing program from crashing

    //Intercept the mode-switch commands to update the Jetson's state
    if (y == -5) {
        tracking_mode_active = true; // Switched to live tracking mode
    } else if (y == -6) {
        tracking_mode_active = false; // Returned to testing mode
    }

    // Prompting output messages to check with data package (i.e object ID and coordinates) are being sent at any given time from the Jetson to the Arduino
    //  std::cout << "Sending Coordinates to the Arduino: Object ID: (" << id << ") -- ";
    //  std::cout << "x: " << x << " y: " << y << std::endl;



    // Construct a packet matching 16-bit integers
    // We send a tiny marker payload: Start byte '!', X bytes, Y bytes, End byte '\n'


    // Note: Standard layout for serial UART information is:    Start Bit -- 8 Bits of info -- Parity Bit (in specifications above always set to 0)-- Stop bit
    // The arduino UNO and Jetson cannot transmit or receive 16 bit binary data of the form XXXXXXXX XXXXXXXX so in stead each set of 8 bits is made into a section stored in sections of "buffer" and then put together again

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

    // Transmit data
    int bytes_written = write(serial_fd, buffer, sizeof(buffer)); // using a built in Linux function called write() this takes the data which was previously re-packaged in the previous steps and writes/sends the information from the Jetson to the Arduino
                                                                  // This function takes in the serial_fd to know which channel the information is being sent on.
                                                                  // It uses buffer to know what the information is that is being sent and uses sizeof(buffer) to know the amount of data that is being sent along the desired channel
    if (bytes_written != sizeof(buffer)) return false; // exits function if the number of bytes written to the arduino is different from the number of bytes stored in buffer

    if (y == -2) {
    // Skip reading here so the telemetry string stays in the serial buffer 
    // waiting for readMotorPosition() to consume it!
    return true; 
    }
    
    // Dynamic if-statement to optimize communication frequency
    // In order to view functionality of the program in VNC viewer, we must read and output to the VNC viewer console what would normally be output by the serial window on the Arduino side so that is what is going to be implemented next here
    // There needs to be a longer time delay for while in testing mode (the else statement) and then performance is optimized with a shorter time delay when analyzing things in tracking mode.
    if (tracking_mode_active) {
        usleep(15000); // 15ms high-speed delay (approx. 66 Hz pipeline capability)
    } else {
        usleep(60000); // 60ms safe delay to clear verbose diagnostic text
    }


    // Create an array of 256 characters (bytes) full of useless info that will be overwritten
    char read_buffer[256];
    memset(read_buffer, 0, sizeof(read_buffer)); // makes all 256 slots in char array become \0 which indicates the end of a string/text when using std::cout -- this makes sure printing and organizing works well

    int bytes_read = read(serial_fd, read_buffer, sizeof(read_buffer) - 1); // input 1: File descriptor showing where USB port connection is
                                                                           // input 2: Linux puts the characters sent back by the Arduino into the read_buffer 256 byte array called read_buffer
                                                                           // input 3: Max number of bytes that can have data stored in it. In this case it is 255 because this matches the number of bytes for our 8 bit unsigned
                                                                           //  integers and also we need the last byte to be the \0 to tell Linux it is done receiving data for that packet.
    
    // bytes_read > 0 is true if ANY bytes/data have been sent to the Arduino so this basically says execute as data is being sent to the Arduino                                                                        
    if (bytes_read > 0) {

        // Output the Arduino's internal processing statements directly to your Jetson console
        // std::cout << "[ARDUINO RESPONSE]:\n" << read_buffer << std::endl;

        // Parsing block to extract the separate variables from the raw text line
        int parsedFrame = 0;
        double parsedPos = 0.0;

        // sscanf searches for fields matching the "SUCCESS: Frame [X] Pos: Y" structure
        if (sscanf(read_buffer, "Frame [%d] Pos: %lf", &parsedFrame, &parsedPos) == 2) {
            // Routes the separate variables directly to the unified logging function
            logTelemetry(logFile, parsedFrame, parsedPos, false);

        } else {
            std::cerr << "[WARNING] Could not parse telemetry variables from line: " << read_buffer << std::endl;
        }

    } else {
        std::cerr << "[WARNING] No verification data returned from Arduino." << std::endl;
    }

    return true; // returns a bool variable value to the system to ensure that the size of the written data is equal to the size of the stored process data. If they are not the same this means that there
                                              // is likely some sort of wire disconnected which stops running the system allowing it to be fixed/debugged


}

/////////////////////////////////////////////////////////////

/* 3.) readMotorPosition() function

Function description: This function reads/receives the current motor position detected by the Arduino sent by the ArduinoReceive side of the code. This data is then sent to the logTelemetry function so that it can store the data in a .txt file later.

Inputs: 
1.) std::ofstream& logFile == This is a reference to an output file stream object that is used to log telemetry data to a text file. It allows the function to write information about the sent coordinates and the current state of the system (tracking or testing mode) to a log text file.

Outputs:
1.) double motorPos == The current motor position detected by the Arduino on the ArduinoReceive side of the code. This is a measurement in radians.

*/


/////////////////////////////////////////////////////////////

double ArduinoSend::readMotorPosition(std::ofstream& logFile) {

    if (serial_fd == -1) return -1.0; // exits the sending coordinates function early if no USB connection descriptor number is found preventing program from crashing

    // Give the Arduino a tiny window (e.g., 20-30ms) to receive the packet,
    // process the motor target change, and reply via Serial.println()
    usleep(25000);

    // Create an array of 256 characters (bytes) full of useless info that will be overwritten
    char read_buffer[256];
    memset(read_buffer, 0, sizeof(read_buffer)); // makes all 256 slots in char array become \0 which indicates the end of a string/text when using std::cout -- this makes sure printing and organizing works well

    int bytes_read = read(serial_fd, read_buffer, sizeof(read_buffer) - 1); // input 1: File descriptor showing where USB port connection is
                                                                            // input 2: Linux puts the characters sent back by the Arduino into the read_buffer 256 byte array called read_buffer
                                                                            // input 3: Max number of bytes that can have data stored in it. In this case it is 255 because this matches the number of bytes for our 8 bit unsigned
                                                                            // integers and also we need the last byte to be the \0 to tell Linux it is done receiving data for that packet.

    if (bytes_read > 0) {
       // std::cout << "[ARDUINO RESPONSE]:\n" << read_buffer << std::endl;

        // Convert the raw bare float response string to a double variable
        double motorPos = std::stod(read_buffer);

        // Routes the motor test position variable to the unified logging function (passing frame as 0)
        logTelemetry(logFile, 0, motorPos, true);

        return motorPos; // Convert string to double and return -- i.e takes in serial print input from the Arduino end and converts it to doubles to be stored later once receiving data back

    } else {
        std::cerr << "[WARNING] No verification data returned from Arduino." << std::endl;
        return -1.0; // Indicate error
    }
}

/////////////////////////////////////////////////////////////

/* 4.) logTelemetry() function

Function description: This function logs telemetry data to a text file. It organizes the data into a structured format based on its type and source. All of this data beeing stored into a .txt file is crucial because it means that analysis can be done
 on all of the data but then the system can dump targets and not have to keep track of too many targets at a given time to optimize performance and speed.

Inputs: 
1.) std::ofstream& logFile == This is a reference to an output file stream object that is used to log telemetry data to a text file. It allows the function to write information about the sent coordinates and the current state of the system (tracking or testing mode) to a log text file.
2.) int frameNum == This is an integer variable that represents a local counter for the Frame number of the camera's video stream. If the Arduino is booted up before the camera then this should exactly match the camera's frame number but if the camera is online first then they will be 
different and there will be a measure to reconcile these numbers.
3.) double motorPos == The current motor position detected by the Arduino on the ArduinoReceive side of the code. This is a measurement in radians.
4.) bool isMotorTest == A boolean flag indicating whether the telemetry data is from a motor test or tracking mode. If true, then it is specifically from a motor test performed in testing mode and if false then it is the live motor position as the system functions and sees targets in tracking mode.

Outputs:
None

*/


/////////////////////////////////////////////////////////////


// Added unified logging function that handles writing the separate metrics into the pre-existing file stream
void ArduinoSend::logTelemetry(std::ofstream& logFile, int frameNum, double motorPos, bool isMotorTest) {
    // Check if the external script's file stream is actually ready to write
    if (!logFile.is_open()) {
        std::cerr << "[ERROR] Target log file stream is not open!" << std::endl;
        return;
    }

    //  Organizes variables into a clean structural column layout based on the data type source
    if (isMotorTest) {
        logFile << "Type: MOTOR_TEST, Frame: N/A, Position: " << motorPos << "\n";
    } else {
        logFile << "Type: TRACKING, Frame: " << frameNum << ", Position: " << motorPos << "\n";
    }

    // Flushes the stream immediately so data hits your disk without waiting for cache boundaries
    logFile.flush();
}

void ArduinoSend::closePort() {
    if (serial_fd != -1) { // checks to see if serial_fd is NOT off (i.e has actually been running and connected) before trying to close it down
        close(serial_fd); // linux function that cuts off the connection to both of the communicating devices 
        serial_fd = -1; // resets the serial value to -1 or OFF so that no duplicate close commands occur
    }
}

void ArduinoSend::flushCache() { // flushes the cache 
    if (serial_fd == -1) return;
    
    char flush_dump[256];
    // Read continuously until the incoming buffer is completely empty
    while (read(serial_fd, flush_dump, sizeof(flush_dump)) > 0) {
        // Just draining the buffer cache
    }
}



