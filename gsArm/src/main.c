#include "hal.h"
#include "gsnode.h"

// URL with info about this Gestalt node
const char* gsNode_url = "http://www.taktia.com/gestalt/nodes/2B1-021";

int main()
{
    // Set up hardware
    hal_init();

    // Set up Gestalt library
    gsNode_init();

    // Application code here
    for(;;);
}

// Called when a Gestalt packet arrives
void gsNode_packetReceived()
{
}

// Various handlers (can be moved out of main if this becomes unwieldy)

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

// Gestalt error handler
void gsNode_error(gsNode_error_t error)
{
    for(;;);
}
