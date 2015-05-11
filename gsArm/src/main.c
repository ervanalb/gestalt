#include "hal.h"
#include "gsnode.h"
#include "motors.h"
#include <string.h>

static const char url[] = "http://www.taktia.com/gestalt/nodes/2B1-021";

void i2c_test();

int main()
{
    // Set up hardware
    hal_init();

    // Set up Gestalt library
    gsNode_address = 0xCCCC;
    gsNode_init();

    //i2c_test();

    /*
    hal_setLED(65535 / 32);

    hal_setXCurrent(65535 / 2);
    hal_setYCurrent(65535 / 2);
    hal_setZCurrent(65535 / 2);
    motor_jog(&motor_x, 65536 * 3200, 65536 / 2);
    motor_jog(&motor_y, 65536 * 3200, 65536 / 2);
    motor_jog(&motor_z, 65536 * 3200, 65536 / 2);
    */

    // Application code here
    for(;;)
    {
        hal_poll();
    }
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

#define SVC_MOVE_TO 10
#define SVC_SET_CURRENT 11
#define SVC_ZERO 12
#define SVC_JOG 13
#define SVC_GET_POSITION 14
#define SVC_GET_BUTTONS 15
#define SVC_SET_LED 20
#define SVC_SET_SOFT_LIMITS 21
#define SVC_GET_LIMITS 22

static void svcMoveTo()
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

static void svcJog()
{
    int32_t xv;
    int32_t yv;
    int32_t zv;

    int32_t t;

    memcpy(&xv, &gsNode_packet.payload[0], sizeof(xv));
    memcpy(&yv, &gsNode_packet.payload[4], sizeof(yv));
    memcpy(&zv, &gsNode_packet.payload[8], sizeof(zv));
    memcpy(&t, &gsNode_packet.payload[12], sizeof(t));

    motor_jog(&motor_x, xv, t);
    motor_jog(&motor_y, yv, t);
    motor_jog(&motor_z, zv, t);

    sendBlankPacket();
}

static void svcGetPosition()
{
    gsNode_packet.address = gsNode_address;
    gsNode_packet.length = sizeof(int32_t) * 3 + 5;
    memcpy(&gsNode_packet.payload[0], &motor_x.p, sizeof(int32_t));
    memcpy(&gsNode_packet.payload[4], &motor_y.p, sizeof(int32_t));
    memcpy(&gsNode_packet.payload[8], &motor_z.p, sizeof(int32_t));

    gsNode_transmitPacket();
}

static void svcZero()
{
    int32_t xp;
    int32_t yp;
    int32_t zp;

    memcpy(&xp, &gsNode_packet.payload[0], sizeof(xp));
    memcpy(&yp, &gsNode_packet.payload[4], sizeof(yp));
    memcpy(&zp, &gsNode_packet.payload[8], sizeof(zp));

    motor_zero(&motor_x, xp);
    motor_zero(&motor_y, yp);
    motor_zero(&motor_z, zp);

    sendBlankPacket();
}

static void svcSetCurrent()
{
    uint16_t xc;
    uint16_t yc;
    uint16_t zc;

    memcpy(&xc, &gsNode_packet.payload[0], sizeof(xc));
    memcpy(&yc, &gsNode_packet.payload[2], sizeof(yc));
    memcpy(&zc, &gsNode_packet.payload[4], sizeof(zc));

    hal_setXCurrent(xc);
    hal_setYCurrent(yc);
    hal_setZCurrent(zc);

    sendBlankPacket();
}

static void svcGetButtons()
{
    gsNode_packet.address = gsNode_address;
    gsNode_packet.length = 2 + 5;
    gsNode_packet.payload[0] = hal_leftButton();
    gsNode_packet.payload[1] = hal_rightButton();

    gsNode_transmitPacket();
}

static void svcSetLED()
{
    uint16_t b;

    memcpy(&b, &gsNode_packet.payload[0], sizeof(b));

    hal_setLED(b);

    sendBlankPacket();
}

static void svcSetSoftLimits()
{
    int32_t x_lower;
    int32_t y_lower;
    int32_t z_lower;
    int32_t x_upper;
    int32_t y_upper;
    int32_t z_upper;

    memcpy(&x_lower, &gsNode_packet.payload[0],  sizeof(x_lower));
    memcpy(&y_lower, &gsNode_packet.payload[4],  sizeof(y_lower));
    memcpy(&z_lower, &gsNode_packet.payload[8],  sizeof(z_lower));
    memcpy(&x_upper, &gsNode_packet.payload[12], sizeof(x_upper));
    memcpy(&y_upper, &gsNode_packet.payload[16], sizeof(y_upper));
    memcpy(&z_upper, &gsNode_packet.payload[20], sizeof(z_upper));

    motor_setSoftLowerLimit(&motor_x, x_lower);
    motor_setSoftLowerLimit(&motor_y, y_lower);
    motor_setSoftLowerLimit(&motor_z, z_lower);

    motor_setSoftUpperLimit(&motor_x, x_upper);
    motor_setSoftUpperLimit(&motor_y, y_upper);
    motor_setSoftUpperLimit(&motor_z, z_upper);
    sendBlankPacket();
}

static void svcGetLimits()
{
    gsNode_packet.address = gsNode_address;
    gsNode_packet.length = 3 + 5;
    gsNode_packet.payload[0] = hal_getLimitsX();
    gsNode_packet.payload[1] = hal_getLimitsY();
    gsNode_packet.payload[2] = hal_getLimitsZ();

    gsNode_transmitPacket();
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

        case SVC_MOVE_TO:
            svcMoveTo();
            break;

        case SVC_SET_CURRENT:
            svcSetCurrent();
            break;

        case SVC_ZERO:
            svcZero();
            break;

       case SVC_JOG:
            svcJog();
            break;

       case SVC_GET_POSITION:
            svcGetPosition();
            break;

       case SVC_GET_BUTTONS:
            svcGetButtons();
            break;

       case SVC_SET_LED:
            svcSetLED();
            break;

       case SVC_SET_SOFT_LIMITS:
            svcSetSoftLimits();
            break;

       case SVC_GET_LIMITS:
            svcGetLimits();
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

