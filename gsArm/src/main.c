#include "hal.h"
#include "gsnode.h"
#include "motors.h"
#include <string.h>

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
    gsNode_packet.address = gsNode_address;
    gsNode_packet.length = 5 + 2;
    gsNode_packet.payload[0] = 'A'; // A for application
    gsNode_packet.payload[1] = 170; // 170 for application good
    gsNode_transmitPacket();
}

static void svcRequestURL()
{
    gsNode_packet.address = gsNode_address;
    gsNode_packet.length = sizeof(url) - 1 + 5;
    memcpy(gsNode_packet.payload, url, sizeof(url) - 1);
    gsNode_transmitPacket();
}

static void svcSetAddress()
{
    memcpy(&gsNode_address, &gsNode_packet.payload[0], sizeof(gsNode_address));
    svcRequestURL();
}

static void sendBlankPacket()
{
    gsNode_packet.address = gsNode_address;
    gsNode_packet.length = 5;
    gsNode_transmitPacket();
}


#define SVC_SET_XY 10
#define SVC_SET_CURRENT 11

static void svcSetXY()
{
    int32_t x;
    int32_t y;
    int32_t z;

    int32_t t;

    memcpy(&x, &gsNode_packet.payload[0], sizeof(x));
    memcpy(&y, &gsNode_packet.payload[4], sizeof(y));
    memcpy(&z, &gsNode_packet.payload[8], sizeof(z));
    memcpy(&t, &gsNode_packet.payload[12], sizeof(t));

    motor_moveTo(&motor_x, x, t);
    motor_moveTo(&motor_y, y, t);
    motor_moveTo(&motor_z, z, t);
}

static void svcSetCurrent()
{
    uint16_t xyc, zc;
    memcpy(&xyc, &gsNode_packet.payload[0], sizeof(xyc));
    memcpy(&zc, &gsNode_packet.payload[4], sizeof(zc));

    hal_setXYCurrent(xyc);
    hal_setZCurrent(zc);

    sendBlankPacket();
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

        case SVC_SET_XY:
            svcSetXY();
            break;

        case SVC_SET_CURRENT:
            svcSetCurrent();
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

