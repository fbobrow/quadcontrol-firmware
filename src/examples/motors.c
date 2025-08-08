#include "FreeRTOS.h"      // FreeRTOS core definitions (needed for task handling and timing)
#include "task.h"          // FreeRTOS task functions (e.g., vTaskDelay)
#include "supervisor.h"    // Functions to check flight status (e.g., supervisorIsArmed)
#include "debug.h"         // Debug printing functions (e.g., DEBUG_PRINT)
#include "motors.h"        // Low-level motor control interface (e.g., motorsSetRatio)

// Main application loop
void appMain(void *param)
{
    // Infinite loop (runs continuously while the drone is powered on)
    while (true)
    {
        // Check if the drone is armed (i.e., ready to receive motor commands)
        if (supervisorIsArmed())
        {
            // If armed, turn on motor 1 with 10% power and print message to console
            DEBUG_PRINT("Turn on motor 1\n");
            motorsSetRatio(MOTOR_M1, 0.1f * UINT16_MAX);
        }
        else
        {
            // If not armed, stop motor 1 and print message to console
            DEBUG_PRINT("Stop motor 1\n");
            motorsSetRatio(MOTOR_M1, 0);
        }
        // Wait for 100 milliseconds before checking again (10 Hz loop)
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
