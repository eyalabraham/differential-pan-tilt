/* 
 * test.c
 *
 *  This program is a test program for the differential pan-tilt
 *  motor controller.
 *
 */

#include    <stdint.h>
#include    <stdio.h>

#include    "v25.h"
#include    "pt-driver.h"

/****************************************************************************
  Module definitions
****************************************************************************/

/****************************************************************************
  Types
****************************************************************************/

/****************************************************************************
  Function prototypes
****************************************************************************/

/****************************************************************************
  Globals
****************************************************************************/

/* ----------------------------------------------------------------------------
 * main() control functions
 *
 */
int main(int argc, char* argv[])
{
    int     i;

    /* IO, timer and interrupt initialization
     */
    io_init();
    timer_isr_init();

    printf("Homing.\n");
    sys_home();
    wait(2000);

    printf("Moving.\n");

    while (1)
        {
            printf("In while(1).\n");

            sys_pan(DIRECTION_CW, 1000, 600);
            sys_wait_all_stop();

            sys_tilt(DIRECTION_CW, 1000, 1200);
            sys_wait_stop(MOTOR_TILT);

            sys_tilt(DIRECTION_CCW, 1000, 1200);
            sys_wait_stop(MOTOR_TILT);
        }

    return 0;
}
