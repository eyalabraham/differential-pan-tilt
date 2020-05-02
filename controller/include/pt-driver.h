/*
 * pt-driver.h
 *
 *  Header file for driver module of the differential pan-tilt motor controller.
 *  NEC V25 ports drive two A4988 stepper motor drivers.
 *
 */

#ifndef __PT_DRIVER_H__
#define __PT_DRIVER_H__

#include    <stdint.h>

#define     MOTOR_MICRO04   0x00    // 1/4 stepping
#define     MOTOR_MICRO08   0x01    // 1/8 stepping
#define     MOTOR_MICRO16   0x03    // 1/16 stepping

typedef enum
    {
        MICROSTEP_1_4  = MOTOR_MICRO04,
        MICROSTEP_1_8  = MOTOR_MICRO08,
        MICROSTEP_1_16 = MOTOR_MICRO16
    } microstep_t;

typedef enum
    {
        DIRECTION_STOP =  0,
        DIRECTION_CW   = +1,
        DIRECTION_CCW  = -1
    } direction_t;

typedef enum
    {
        MOTOR_PAN  = 0,
        MOTOR_TILT = 1
    } motor_t;


// Driver setup and utilities
void     io_init(void);                                       // Initialize IO ports (must be called first!)
void     timer_isr_init(void);                                // Initialize timer and interrupts (must be called second!)
void     wait(uint16_t);                                      // Timed wait (block) in mSec

// Motor controls
//void     motor_set_microstep(motor_t, microstep_t);           // Setup motor micro-step setting
int      motor_set_direction(motor_t, direction_t);           // Change motor turn direction: CW, CCW, or stop
int      motor_set_rate(motor_t, int);                        // Set motor speed in steps per second
int      motor_run(motor_t, direction_t, int, int);           // Run motor in a direction at set rate optional step count
int      motor_get_position(motor_t);                         // Return motor step position relative to home

#define  motor_stop(m)   motor_set_direction(m, DIRECTION_STOP);
#define  motor_cw(m)     motor_set_direction(m, DIRECTION_CW);
#define  motor_ccw(m)    motor_set_direction(m, DIRECTION_CCW);

// System controls
void     sys_home(void);                                      // Home position for pan and tilt
int      sys_pan(direction_t, int, int);                      // Pan in direction at rate with optional step count
int      sys_tilt(direction_t, int, int);                     // Tilt in direction at rate with optional step count
void     sys_wait_stop(motor_t);                              // Wait for motor to stop moving
void     sys_wait_all_stop(void);                             // Wait for all motors to stop moving
uint16_t sys_a2d_read(uint8_t);                               // Read and D-to-A channel
int      sys_get_microsteps(void);                            // Get micro-step setting (hard-coded)
int      sys_limit_switch(void);                              // Return state of limit switch

#endif  /* __PT_DRIVER_H__ */
