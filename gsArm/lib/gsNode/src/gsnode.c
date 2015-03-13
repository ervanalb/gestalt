#include "gsnode.h"
#include "gsnode_hal.h"

gsNode_packet_t gsNode_packet;

static enum {IDLE, RX, TX} state;

static uint8_t byteIndex;
static uint8_t crc;

uint16_t gsNode_badPacketCounter;
uint16_t gsNode_timeoutCounter;
uint16_t gsNode_receiveWhileTransmitCounter;

#include "crc.inc" // provides crcTable

static void reset()
{
    crc = 0;
    byteIndex = 0;
    state = IDLE;
}

void gsNode_init()
{
    gsNode_badPacketCounter = 0;
    gsNode_timeoutCounter = 0;
    reset();
    gsNode_hal_readByte();
}

static void write_next_byte()
{
    uint8_t byte;

    if(byteIndex == 0) // Start byte
    {
        byte = gsNode_packet.start;
    }
    else if(byteIndex == 1) // First byte of address
    {
        byte = ((uint8_t*)(&(gsNode_packet.address)))[0];
    }
    else if(byteIndex == 2) // Second byte of address
    {
        byte = ((uint8_t*)(&(gsNode_packet.address)))[1];
    }
    else if(byteIndex == 3) // Port
    {
        byte = gsNode_packet.port;
    }
    else if(byteIndex == 4) // Length
    {
        byte = gsNode_packet.length;
    }
    else if(byteIndex < gsNode_packet.length) // Payload
    {
        byte = gsNode_packet.payload[byteIndex - 5];
    }
    else
    {
        byte = crc;
    }

    byteIndex++;
    crc = crcTable[crc ^ byte];
    gsNode_hal_writeByte(byte);
}

// Transmits the packet currently stored in gsNode_packet
void gsNode_transmitPacket()
{
    state = TX;
    write_next_byte();
}

// Callback functions

void gsNode_hal_byteWritten()
{
    if(byteIndex > gsNode_packet.length) // if the packet is complete
    {
        gsNode_hal_flushOutput();
    }
    else
    {
        write_next_byte();
    }
}

void gsNode_hal_outputFlushed()
{
    reset();
}

void gsNode_hal_byteRead(uint8_t byte)
{
    if(state == TX)
    {
        gsNode_receiveWhileTransmitCounter++;
        gsNode_hal_readByte();
        return;
    }

    state = RX;

    if(byteIndex == 0) // Start byte
    {
        gsNode_packet.start = byte;
    }
    else if(byteIndex == 1) // First byte of address
    {
        ((uint8_t*)(&(gsNode_packet.address)))[0] = byte;
    }
    else if(byteIndex == 2) // Second byte of address
    {
        ((uint8_t*)(&(gsNode_packet.address)))[1] = byte;
    }
    else if(byteIndex == 3) // Port
    {
        gsNode_packet.port = byte;
    }
    else if(byteIndex == 4) // Length
    {
        gsNode_packet.length = byte;
    }
    else if(byteIndex < gsNode_packet.length) // Payload
    {
        gsNode_packet.payload[byteIndex - 5] = byte;
    }

    byteIndex++;
    crc = crcTable[crc ^ byte];

    if(byteIndex > 4 && byteIndex > gsNode_packet.length) // Packet is over
    {
        if(crc == 0)
        {
            reset();
            gsNode_packetReceived();
        }
        else
        {
            gsNode_badPacketCounter++;
            reset();
        }
    }

    if(state == RX)
    {
        gsNode_hal_setTimer();
    }

    gsNode_hal_readByte();
}

void gsNode_hal_timerFired()
{
    gsNode_timeoutCounter++;
    reset();
}

