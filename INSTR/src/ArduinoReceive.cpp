#ifndef ARDUINO_RECEIVE_HPP
#define ARDUINO_RECEIVE_HPP

#include <Arduino.h> // Essential for standard Arduino data types (uint16_t, int16_t, etc.)

// ============================================================================
// GLOBAL STATE VARIABLES
// ============================================================================
// 'extern' tells the compiler that these variables exist and are shared 
// across multiple files, preventing double-definition linker conflicts.
extern uint16_t activeTargetID;
extern int16_t  targetXCoord;
extern int16_t  targetYCoord;

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================

/**
 * @brief Continually reads from the serial buffer, parses the 8-byte binary stream,
 * and updates tracking coordinates upon validating packet alignment frame markers.
 */
void processIncomingSerial();

/**
 * @brief Computes mathematical PID loops, error deltas, and actuates physical 
 * hardware states using the newly parsed synchronized target data.
 * * @param id The 16-bit target identification tag.
 * @param x  The raw 16-bit horizontal coordinate value.
 * @param y  The raw 16-bit vertical coordinate value.
 */
void executePIDControl(uint16_t id, int16_t x, int16_t y);

#endif // ARDUINO_RECEIVE_HPP
