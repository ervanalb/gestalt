#ifndef __GSNODE_H
#define __GSNODE_H

// Gestalt Physical Node Library

#include <stdint.h>

#define GSNODE_PAYLOAD_LENGTH 250

typedef struct {
    uint8_t start;
    uint16_t address;
    uint8_t port;
    uint8_t length;
    uint8_t payload[GSNODE_PAYLOAD_LENGTH];
} gsNode_packet_t;

// Functions provided by the library
extern gsNode_packet_t gsNode_packet;
extern uint16_t gsNode_address;

// Error counters
extern uint16_t gsNode_badPacketCounter;
extern uint16_t gsNode_timeoutCounter;
extern uint16_t gsNode_receiveWhileTransmitCounter;

void gsNode_init();
void gsNode_transmitPacket();

// Functions that need to be implemented by the application

// Called when a complete packet is received
void gsNode_packetReceived();

#define GSNODE_SVC_STATUS               1
#define GSNODE_SVC_BOOTLOADER_COMMAND   2
#define GSNODE_SVC_BOOTLOADER_DATA_PORT 3
#define GSNODE_SVC_BOOTLOADER_READ_PORT 4
#define GSNODE_SVC_REQUEST_URL          5
#define GSNODE_SVC_SET_ADDRESS          6
#define GSNODE_SVC_IDENTIFY_NODE        7
#define GSNODE_SVC_RESET_NODE           8

#endif
