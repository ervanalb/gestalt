#include "gsnode.h"
#include "gsnode_hal.h"

gsNode_packet_t gsNode_packet;

static enum {IDLE, RX, TX} state; // It is assumed that writes to state are atomic

static uint8_t byte_index;

void gsNode_init()
{
    // Set up state machine
    // Call readByte
}

void gsNode_transmitPacket()
{
    // Set state machine to TX and write a byte
}

// Callback functions

void gsNode_hal_byteWritten()
{
    // Write another byte or go to IDLE
}

void gsNode_hal_outputFlushed()
{
    // Set state machine to IDLE
}

void gsNode_hal_byteRead(uint8_t byte)
{
    // If done, call gsNode_packetReceived() and move to IDLE
    // Otherwise, update packet and call void gsNode_hal_setTimer()
    // Call gsNode_hal_readByte() at the end regardless
}

void gsNode_hal_timerFired()
{
    // If state machine is RX, set state machine to IDLE and reset
}

