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
    uint8_t checksum;
} gsNode_packet_t;

// An error type indicating what went wrong
typedef enum {GSNODE_ERROR_GENERIC} gsNode_error_t;

// Functions provided by the library
extern gsNode_packet_t gsNode_packet;

void gsNode_init();
void gsNode_transmitPacket();

// Functions that need to be implemented by the application

// Should contain the node's URL
extern const char* gsNode_url;

// Called when a complete packet is received
void gsNode_packetReceived();

// Called when an error is encountered
void gsNode_error(gsNode_error_t error);

#endif
