#ifndef __GSNODE_HAL_H
#define __GSNODE_HAL_H

// Gestalt Physical Node Hardware Abstraction Layer
// Implementing this interface allows your platform to be compatible with
// the gsNode library. Simply define each function described here and gsNode
// will take care of the rest.

// An asynchronous paradigm is used, which may be implemented either through
// polling, interrupts, or a combination of both.

#include <stdint.h>

// Constants
#define GSNODE_TIMER_US 1000

// Functions to implement
// (these functions have no definition in the library, you must define them!)

// Write out a byte
void gsNode_hal_writeByte(uint8_t byte);
// once the byte is written, call gsNode_byteWritten()

// Flush the TX buffer
void gsNode_hal_flushOutput();
// once the buffer is flushed, call gsNode_hal_outputFlushed()

// Enable reception of data
void gsNode_hal_readByte();
// once a byte has been read, call gsNode_hal_byteRead()

// Set a timer for GSNODE_TIMER_US microseconds
void gsNode_hal_setTimer();
// once the timer expires, call gsNode_hal_timerFired()

// Cancel the current timer (prevent calling of gsNode_hal_timerFired)
void gsNode_hal_cancelTimer();

// Callback functions
// (these functions are defined in the library, but you must call them at the
// appropriate times for the library to work!)
void gsNode_hal_byteWritten();
void gsNode_hal_outputFlushed();
void gsNode_hal_byteRead(uint8_t byte);
void gsNode_hal_timerFired();

#endif
