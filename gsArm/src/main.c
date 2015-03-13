#include "hal.h"
#include "gsnode.h"

static const char url[] = "http://www.taktia.com/gestalt/nodes/2B1-021";

int main()
{
    // Set up hardware
    hal_init();

    // Set up Gestalt library
    gsNode_address = 0xCCCC;
    gsNode_init();

    // Application code here
    for(;;);
}

// Service functions to handle incoming standard packets
static void svcStatus()
{
    gsNode_packet.length = 2;
    gsNode_packet.payload[0] = 'A'; // A for application
    gsNode_packet.payload[1] = 170; // 170 for application good
    gsNode_transmitPacket();
}

static void svcRequestURL()
{
    gsNode_packet.length = sizeof(url)-1;
    memcpy(gsNode_packet.payload, url, sizeof(url)-1);
    gsNode_transmitPacket();
}

static void svcSetAddress()
{
    gsNode_address = *(uint16_t*)&gsNode_packet.payload[0];
    svcRequestURL();
}

// Called when a Gestalt packet arrives
void gsNode_packetReceived()
{
    switch(gsNode_packet.port)
    {
        case GSNODE_SVC_STATUS:
            svcStatus();
            break;

        case GSNODE_SVC_REQUEST_URL:
            svcRequestURL();
            break;

        case GSNODE_SVC_SET_ADDRESS:
            svcSetAddress();
            break;

        // Future
        //case GSNODE_SVC_BOOTLOADER_COMMAND:
        //case GSNODE_SVC_BOOTLOADER_DATA_PORT:
        //case GSNODE_SVC_BOOTLOADER_READ_PORT:
        //case GSNODE_SVC_IDENTIFY_NODE:
        //case GSNODE_SVC_RESET_NODE:
        default:
            // Unhandled packet
            break;
    }
}

// Various system handlers

// For standard peripheral library assert statements
// (used in conjunction with #define DEBUG)
void assert_failed(const char* file, const int line)
{
    for(;;);
}

// Processor hard fault handler
void HardFault_Handler(void)
{
    for(;;);
}

