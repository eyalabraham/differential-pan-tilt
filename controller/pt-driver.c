/* 
 * pt-driver.c
 *
 *  Driver module for the differential pan-tilt motor controller.
 *  NEC V25 ports drive two A4988 stepper motor drivers.
 * 
 * Port-0 bit assignment
 *
 *  b7 b6 b5 b4 b3 b2 b1 b0
 *  |  |  |  |  |  |  |  |
 *  |  |  |  |  |  |  |  +--- 'o' Pan  step pulse
 *  |  |  |  |  |  |  +------ 'o' Pan  direction 0=CW, 1=CCW
 *  |  |  |  |  |  +--------- 'o' Tilt step pulse
 *  |  |  |  |  +------------ 'o' Tilt direction 0=CW, 1=CCW
 *  |  |  |  +--------------- 'i'
 *  |  |  +------------------ 'i'
 *  |  +--------------------- 'i'
 *  +------------------------ 'o' Interrupt time-base test point
 *
 * Port-1 bit assignment
 *
 *  b7 b6 b5 b4 b3 b2 b1 b0
 *  |  |  |  |  |  |  |  |
 *  |  |  |  |  |  |  |  +--- 'i'
 *  |  |  |  |  |  |  +------ 'i'
 *  |  |  |  |  |  +--------- 'i'
 *  |  |  |  |  +------------ 'i'
 *  |  |  |  +--------------- 'o' A/D ^CS  (A/D pin.18)
 *  |  |  +------------------ 'o' A/D SCLK (A/D pin.19)
 *  |  +--------------------- 'o' A/D DIN  (A/D pin.17)
 *  +------------------------ 'i' A/D DOUT (A/D pin.15)
 *
 * Port-2 bit assignment
 *
 *  b7 b6 b5 b4 b3 b2 b1 b0
 *  |  |  |  |  |  |  |  |
 *  |  |  |  |  |  |  |  +--- 'o' Pan  MS0  00=1/4, 10=1/8, 11=1/16
 *  |  |  |  |  |  |  +------ 'o'      MS1
 *  |  |  |  |  |  +--------- 'o' Tilt MS0  00=1/4, 10=1/8, 11=1/16
 *  |  |  |  |  +------------ 'o'      MS1
 *  |  |  |  +--------------- 'i'
 *  |  |  +------------------ 'i'
 *  |  +--------------------- 'i' Pan home limit switch
 *  +------------------------ 'i' A/D SSTRB (A/D pin.16)
 *
 * TODO
 * 1. sys_pan() and sys_tilt() need to be modified to account for the
 *    relationship between pan and tilt movement i.e. pan motor movement
 *    changes tilt angle according to:
 *    Po = P
 *    To = T - P
 * 2. For #1 above, step counts in drv-isr.asm need to be adjusted to account
 *    for steps in a "blended" motion of pan and/or tilt
 * 3. Add soft limits by step counts in drv-isr.asm after correcting #2
 * 4. Ability to set micro-step configuration to replace hard-coded configuration
 * 5. Fix step counts: HOME > CCW 600 > CW 1200 > CCW 600 results in step 1
 *                     HOME > CCW 600 > CW 600 results in 0
 *                     HOME > CCW 500 results in -599
 */

#include    <stdlib.h>
#include    <stdint.h>
#include    <limits.h>

#include    "v25.h"
#include    "pt-driver.h"

/****************************************************************************
  Module definitions
****************************************************************************/
#define     __VERSION__     "1.0"

// Driver micro-step hard-coded
#define     MICRO_STEP      8       // 4, 8, or 16

#if (MICRO_STEP == 4)
#define     P2_STEP_INIT    (MOTOR_MICRO04 + (MOTOR_MICRO04 << MOTOR_TILT_SEL))
#elif (MICRO_STEP == 8)
#define     P2_STEP_INIT    (MOTOR_MICRO08 + (MOTOR_MICRO08 << MOTOR_TILT_SEL))
#elif (MICRO_STEP == 16)
#define     P2_STEP_INIT    (MOTOR_MICRO16 + (MOTOR_MICRO16 << MOTOR_TILT_SEL))
#else
#error "MICRO_STEP definition error!"
#endif

// IO ports 0, 1 and 2 initialization
#define     P0_MODE_CTL     0x00
#define     P0_MODE         0x70
#define     P0_INIT         0x00

#define     P1_MODE_CTL     0x00
#define     P1_MODE         0x8f
#define     P1_INIT         (AD_CS) // ^CS asserted

#define     P2_MODE_CTL     0x00
#define     P2_MODE         0xf0
#define     P2_INIT         P2_STEP_INIT

// Bit masks
#define     MOTOR_STEP      0x01
#define     MOTOR_DIR_CCW   0x02    // OR for CCW, AND with complement for CW
#define     MOTOR_MICRO_CLR 0x03    // Clear micro-step selection bits (AND with complement)

#define     MOTOR_PAN_SEL   0       // Control-bit shift values
#define     MOTOR_TILT_SEL  2

#define     PAN_LIMIT_SW    0x40    // Pan limit switch
#define     HEART_BEAT      0x80    // Toggle heart beat

// Timer0 configuration
#define     TIMER_CTRL      0x80    // Timer0 interval mode
#define     TIMER_INT_MASK  0x07
#define     TIMER_DIV       332     // Timer setup for 5KHz count on NEC SBC running 10Mhz

#define     CPU_F_CLK       10000   // *** CPU clock in KHz ***
#define     TIMER_CLK_DIV   6
#define     CYCLE_CONST     ((uint16_t)(CPU_F_CLK / TIMER_CLK_DIV / (TIMER_DIV + 1)))

#define     TIM0_IRQ_VECT   28      // Timer0 interrupt vector

// A-to-D definitions
#define     AD_CONTROL      0x8e    // Uni-polar, single-ended, internal-clock conversion

#define     AD_CS           0x10    // Outputs to A-to-D
#define     AD_SCLK         0x20
#define     AD_SER_DATA_IN  0x40

#define     AD_SER_DATA_OUT 0x80    // Inputs from A-to-D
#define     AD_SSTRB        0x80

// System definitions
#define     MOTOR_COUNT     2

#define     HOME_POSITION   2580    // Home position (equal to 2.580v on turret potentiometer)

#define     STEPS_PER_REV   (600 * MICRO_STEP)
#define     STEPS_PER_DEG   (STEPS_PER_REV / 360)

#define     RATE_LIMIT      2500    // Max step rate (mechanical limit)
#define     TILT_HOME_RATE  150     // Steps per second

/* *** To modify these definitions only change the degrees setting ***
 */
#define     PAN_LIMIT_CW    ( 120.0 * STEPS_PER_DEG) // In steps
#define     PAN_LIMIT_CCW   (-120.0 * STEPS_PER_DEG)
#define     TILT_LIMIT_CW   (  90.0 * STEPS_PER_DEG)
#define     TILT_LIMIT_CCW  ( -90.0 * STEPS_PER_DEG)
#define     TILT_HOME_RESET ( 109.0 * STEPS_PER_DEG) // Steps from limit switch to top position

/****************************************************************************
  Types
****************************************************************************/

/* Direction convention: '-1' to CCW, '+1' to CW
 */
typedef struct
    {
        int      rate;          // Motor run rate steps per second [1..RATE_LIMIT]
        uint16_t isr_rate;      // ISR step pulse cycle
        uint16_t counter;       // Motor is stepped when counter == 0, decrement every time ISR runs
        int      dir;           // Direction, '-1' to CCW, '+1' to CW, '0' stop
        int      steps;         // Steps to move, if '-1' then move until stopped or limit reached
        int      curr_pos;      // Current position
        int      limit_high;    // High limit step count
        int      limit_low;     // Low limit step count
        uint8_t  step_ctrl_bit; // Motor driver 'step' IO pin
        uint8_t  dummy;         // Structure alignment byte
    } stepctrl_t;

/****************************************************************************
  Function prototypes
****************************************************************************/

/****************************************************************************
  External function prototypes
****************************************************************************/

/* ---------------------------------------------------------
   isr()

   Interrupt service routine.
   Invoked every clock tick and sequences stepper motors.
--------------------------------------------------------- */
extern void driver_isr(void);

/****************************************************************************
  Globals
****************************************************************************/
static struct SFR __far *pSfr;

volatile uint16_t   timer_ticks = 0;    // clock timer ticks, increment at PID_FREQ [Hz]
volatile stepctrl_t motors[MOTOR_COUNT] = {
    {0, 0, 0, DIRECTION_STOP, -1, 0, PAN_LIMIT_CW,  PAN_LIMIT_CCW,  (MOTOR_STEP << MOTOR_PAN_SEL),  0}, // 0 - Pan
    {0, 0, 0, DIRECTION_STOP, -1, 0, TILT_LIMIT_CW, TILT_LIMIT_CCW, (MOTOR_STEP << MOTOR_TILT_SEL), 0}, // 1 - Tilt
};

/* ----------------------------------------------------------------------------
 * io_init()
 *
 *  Initialize IO port pins
 *
 * param:   nothing
 * return:  nothing
 *
 */
void io_init(void)
{
    /* Global pointer to SFR
     */
    pSfr = MK_FP(0xf000, 0xff00);

    /* Output vital register settings
     */
/*
    printf("PRC=0x%02x\n", pSfr->prc);
    printf("WTC=0x%02x\n", pSfr->wtc);
    printf("IDB=0x%02x\n", pSfr->idb);
*/

    /* Setup IO ports
     */
    pSfr->portmc0 = P0_MODE_CTL;
    pSfr->portm0  = P0_MODE;
    pSfr->port0   = P0_INIT;

    pSfr->portmc1 = P1_MODE_CTL;
    pSfr->portm1  = P1_MODE;
    pSfr->port1   = P1_INIT;

    pSfr->portmc2 = P2_MODE_CTL;
    pSfr->portm2  = P2_MODE;
    pSfr->port2   = P2_INIT;
}

/* ---------------------------------------------------------
   timer_isr_init()

   This function sets up the timer and interrupt vectors.
   The timer is set to interrupt every 'wMiliSecInterval'
   mili-seconds.
--------------------------------------------------------- */
void timer_isr_init(void)
{
 uint16_t __far *isr_vector;

 _disable();

 pSfr->tmc1  &= 0x7f;                   // Stop timer-1
 pSfr->tmic1 |= 0x40;                   // Mask timer-1 interrupts

 /* setup interrupt vector
  */
 isr_vector      = MK_FP(0, (TIM0_IRQ_VECT * 4));
 *isr_vector++   = FP_OFF(driver_isr);
 *isr_vector     = FP_SEG(driver_isr);

 /* setup interrupt controller and timer
  */
 pSfr->tmc0   = (TIMER_CTRL & 0x7f);    // Interval timer, now stopped
 pSfr->md0    = TIMER_DIV;
 pSfr->tmic0 &= TIMER_INT_MASK;         // Timer0 vector interrupt
 pSfr->tmc0   = TIMER_CTRL;             // Start timer

 _enable();
}

/* ----------------------------------------------------------------------------
 * wait()
 *
 * Blocking delay function based on clock tick counter.
 * The maximum is determined by the ISR rate and max value of uint16
 * as: _UI16_MAX / CYCLE_CONST
 *
 * param:   msec
 * return:  nothing
 *
 */
void wait(uint16_t msec)
{
    uint16_t    delta_time;
    uint16_t    current_time;

    if ( msec > (_UI16_MAX / CYCLE_CONST) )
        msec = (_UI16_MAX / CYCLE_CONST);

    current_time = timer_ticks;
    delta_time = CYCLE_CONST * msec;

    while ( (timer_ticks - current_time) < delta_time );
}

/* ----------------------------------------------------------------------------
 * motor_set_microstep()
 *
 * Set motor micro-step rate.
 * The function affect the motor IO port bits.
 *
 * param:   Motor and micro-step selection
 * return:  nothing
 *
 */
/*
void motor_set_microstep(motor_t motor, microstep_t micro_step)
{
    uint8_t micro_step_setting;
    int     ctrl_bit_shift;

    ctrl_bit_shift = (motor == MOTOR_PAN) ? MOTOR_PAN_SEL : MOTOR_TILT_SEL;

    micro_step_setting = pSfr->port2 & ~(MOTOR_MICRO_CLR << ctrl_bit_shift);
    micro_step_setting |= (micro_step << ctrl_bit_shift);
    pSfr->port2 = micro_step_setting;
}
*/

/* ----------------------------------------------------------------------------
 * motor_set_direction()
 *
 * Set motor turn direction rate or stop.
 * The function affects the motor IO port bits and the global array motors[].
 *
 * param:   Motor and turn direction selection
 * return:  '0' if parameters ok, '-1' if error in parameters
 *
 */
int motor_set_direction(motor_t motor, direction_t direction)
{
    int     current_steps;
    int     current_rate;

    _disable();
    current_steps = motors[motor].steps;
    current_rate = motors[motor].rate;
    _enable();

    return motor_run(motor, direction, current_rate, current_steps);
}

/* ----------------------------------------------------------------------------
 * motor_set_rate()
 *
 * Set motor turn rate.
 * The function the global array motors[].
 *
 * param:   Motor and turn rate in steps per second
 * return:  '0' if parameters ok, '-1' if error in parameters
 *
 */
int motor_set_rate(motor_t motor, int rate)
{
    int         current_steps;
    direction_t current_direction;

    _disable();
    current_steps = motors[motor].steps;
    current_direction = motors[motor].dir;
    _enable();

    return motor_run(motor, current_direction, rate, current_steps);
}

/* ----------------------------------------------------------------------------
 * motor_run()
 *
 * Run motor in a direction at set rate with optional step count.
 * The function affects the global array motors[].
 * A 'rate' of zero stops the motor, but does not change
 * current rate settings or remaining step count.
 *
 * param:   Motor, turn direction and step rate in steps per second.
 *          Optional step count if greater than 0, continuous run if steps is '-1'
 * return:  '0' if parameters ok, '-1' if error in parameters
 *
 */
int motor_run(motor_t motor, direction_t direction, int rate, int steps)
{
    int     ctrl_bit_shift;

    if ( motor != MOTOR_PAN && motor != MOTOR_TILT )
        return -1;

    if ( rate > RATE_LIMIT || rate < 0 )
        return -1;

    if ( rate == 0 )
    {
        motors[motor].dir = DIRECTION_STOP;
        return 0;
    }

    if ( steps < -1 )
        return -1;

    ctrl_bit_shift = (motor == MOTOR_PAN) ? MOTOR_PAN_SEL : MOTOR_TILT_SEL;

    _disable();

    /* Initialize step rate
     */
    motors[motor].rate = rate;
    motors[motor].isr_rate = (CYCLE_CONST * 1000) / rate;
    motors[motor].counter = 1;

    /* Initialize step count if given
     */
    motors[motor].steps = steps;

    /* Set direction and run motor
     */
    motors[motor].dir = direction;

    if ( direction == DIRECTION_CCW )
    {
        pSfr->port0 &= ~(MOTOR_DIR_CCW << ctrl_bit_shift);
    }
    else
    {
        pSfr->port0 |= (MOTOR_DIR_CCW << ctrl_bit_shift);
    }

    _enable();

    return 0;
}


/* ----------------------------------------------------------------------------
 * motor_get_position()
 *
 * Return a motor's position in relative steps from home.
 *
 * param:   Motor number
 * return:  Step count
 *
 */
int motor_get_position(motor_t motor)
{
    return motors[motor].curr_pos;
}

/* ----------------------------------------------------------------------------
 * sys_home()
 *
 * Move pan and tilt axis to home position.
 * First pan to home, then tilt to micro-switch and back to to position.
 * Pan analog position is read from a 360-deg position potentiometer,
 * and home point is controlled with a simple PID loop.
 *
 * TODO Extract turret position PID to a general function
 *
 * param:   nothing
 * return:  nothing
 *
 */
#define     HOME_P      5
#define     HOME_I      0
#define     HOME_D      0

void sys_home(void)
{
    int         turn_dir;
    uint16_t    position;
    uint16_t    rate, position_err, position_err_integ, position_err_diff;

    rate = 0;
    position_err = 0;
    position_err_integ = 0;
    position_err_diff = 0;

    motor_stop(MOTOR_PAN);
    motor_stop(MOTOR_TILT);

    /* Bogus motor limits to allow homing
     */
    _disable();
    motors[MOTOR_PAN].limit_high = _I16_MAX;
    motors[MOTOR_PAN].limit_low = _I16_MIN;
    motors[MOTOR_TILT].limit_high = _I16_MAX;
    motors[MOTOR_TILT].limit_low = _I16_MIN;
    _enable();

    /* *** PID is tuned for 8 micro steps ***
     */
    for (;;)
    {
        position = sys_a2d_read(0);

        /* Determine pan position and turn direction
         * towards home, exit when home position is reached
         */
        if ( position < HOME_POSITION  )
            turn_dir = DIRECTION_CCW;
        else if ( position > HOME_POSITION )
            turn_dir = DIRECTION_CW;
        else
            break;

        /* Calculate PID
         */
        position_err = abs(position - HOME_POSITION);
        position_err_diff = position_err - position_err_diff;
        position_err_integ += position_err;

        if ( position_err_integ > 100 )
            position_err_integ = 100;

        rate = (position_err  * HOME_P) + (position_err_integ * HOME_I) + (position_err_diff * HOME_D);

        position_err_diff = position_err;

        if ( rate == 0 )        // Prevent loop from hanging, see motor_run()
            rate = 1;
        else if ( rate > (RATE_LIMIT - 500) ) // Limit to avoid mechanical issues
            rate = (RATE_LIMIT - 500);

        /* Set turret motors to pan into position
         * without tilting
         */
        motor_run(MOTOR_PAN, turn_dir, rate, -1);
        motor_run(MOTOR_TILT, turn_dir, rate, -1);

        wait(50);
    }

    motor_stop(MOTOR_PAN);
    motor_stop(MOTOR_TILT);

    /* Home the tilt axis
     */
    motor_run(MOTOR_TILT, DIRECTION_CW, TILT_HOME_RATE, -1);

    while ( (pSfr->port2 & PAN_LIMIT_SW) )
    {
        wait(10);
    }

    motor_stop(MOTOR_TILT);

    motor_run(MOTOR_TILT, DIRECTION_CCW, (2 * TILT_HOME_RATE), TILT_HOME_RESET);

    sys_wait_stop(MOTOR_TILT);  // Block until motor reaches starting position

    _disable();

    /* Set home position
     */
    motors[MOTOR_PAN].curr_pos = 0;
    motors[MOTOR_TILT].curr_pos = 0;

    /* Restore motor limits
     */
    motors[MOTOR_PAN].limit_high = PAN_LIMIT_CW;
    motors[MOTOR_PAN].limit_low = PAN_LIMIT_CCW;
    motors[MOTOR_TILT].limit_high = TILT_LIMIT_CW;
    motors[MOTOR_TILT].limit_low = TILT_LIMIT_CCW;

    _enable();
}

/* ----------------------------------------------------------------------------
 * sys_pan()
 *
 * Pan in direction at rate with optional step count.
 *
 * param:   Pan direction and step rate in steps per second.
 *          Optional step count if greater than 0, continuous run if steps is '-1'
 * return:  '0' if parameters ok, '-1' if error in parameters
 *
 */
int sys_pan(direction_t direction, int rate, int steps)
{
    int     result = 0;

    result = motor_run(MOTOR_PAN, direction, rate, steps);

    if ( result == 0)
    {
        result = motor_run(MOTOR_TILT, direction, rate, steps);
        if ( result != 0 )
        {
            motor_stop(MOTOR_PAN);  // Stop pan motor if cannot tilt
        }
    }

    return result;
}

/* ----------------------------------------------------------------------------
 * sys_tilt()
 *
 * Tilt in direction at rate with optional step count
 *
 * param:   Tilt direction and step rate in steps per second.
 *          Optional step count if greater than 0, continuous run if steps is '-1'
 * return:  '0' if parameters ok, '-1' if error in parameters
 *
 */
int sys_tilt(direction_t direction, int rate, int steps)
{
    return motor_run(MOTOR_TILT, direction, rate, steps);
}

/* ----------------------------------------------------------------------------
 * sys_wait_stop()
 *
 * Wait (block) until specified motor stops turning.
 *
 * param:   Motor
 * return:  nothing
 *
 */
void sys_wait_stop(motor_t motor)
{
    while ( motors[motor].dir != 0 )
    {
        wait(100);
    }
}

/* ----------------------------------------------------------------------------
 * sys_wait_all_stop()
 *
 * Wait (block) for all motors to stop moving.
 *
 * param:   nothing
 * return:  nothing
 *
 */
void sys_wait_all_stop(void)
{
    while ( motors[MOTOR_PAN].dir != 0 || motors[MOTOR_TILT].dir != 0 )
    {
        wait(100);
    }
}

/* ----------------------------------------------------------------------------
 * sys_a2d_read()
 *
 * Read an A-to-D channel.
 * This function blocks until a conversion is ready, and then returns the 12-bit
 * A-to-D converter value.
 * The A-to-D is a MAX186 SPI device, and the function runs a 'bit-banging'
 * SPI channel to setup, start, and read the conversion.
 * Function assumes that IO ports are configured!
 *
 * param:   A-to-D channel to read, one of eight channels 0 to 7
 * return:  12-bit A-to-D conversion
 *
 */
uint16_t sys_a2d_read(uint8_t channel)
{
    uint8_t     i;
    uint8_t     control_byte;
    uint16_t    data_word;

    if ( channel > 7 )
        return 0;

    /* Send control byte to select channel and
     * start the conversion
     */
    control_byte = AD_CONTROL | (channel << 4);
    pSfr->port1 &= ~AD_CS;
    for ( i = 0; i < 8; i++ )
    {
        if ( (control_byte & 0x80) == 0 )   // MSB first
        {
            pSfr->port1 &= ~AD_SER_DATA_IN;
        }
        else
        {
            pSfr->port1 |= AD_SER_DATA_IN;
        }

        pSfr->port1 |= AD_SCLK;
        pSfr->port1 &= ~AD_SCLK;

        control_byte = control_byte << 1;
    }

    pSfr->port1 &= ~AD_SER_DATA_IN;
    pSfr->port1 |= AD_CS;

    /* Wait for SSTRB to go high
     * TODO Implement a time-out?
     */
    while ( !(pSfr->port2 & AD_SSTRB) ) {}

    /* Read 16-bit value from A-to-D
     */
    data_word = 0;
    pSfr->port1 &= ~AD_CS;
    for ( i = 16; i > 0; i-- )
    {
        pSfr->port1 |= AD_SCLK;
        pSfr->port1 &= ~AD_SCLK;

        if ( i < 5 )                        // Discard last four bits
            continue;
        else if ( (pSfr->port1 & AD_SER_DATA_OUT) )
        {
            data_word |= ( (uint16_t)1 << (i - 5));
        }
    }

    pSfr->port1 |= AD_CS;

    data_word &= 0x0fff;

    return data_word;
}

/* ----------------------------------------------------------------------------
 * sys_get_microsteps()
 *
 * Get hard-coded micro-step setting.
 *
 * param:   nothing
 * return:  micro-step setting
 *
 */
int sys_get_microsteps(void)
{
    return MICRO_STEP;
}

/* ----------------------------------------------------------------------------
 * sys_limit_switch()
 *
 * Return state of limit switch.
 * IO ports must be preinitialized.
 *
 * param:   nothing
 * return:  0 = switch open, not 0 = switch pressed
 *
 */
int sys_limit_switch(void)
{
    uint16_t    i;
    int         switch_state1, switch_state2;

    /* Debounce
     */
    switch_state1 = (int)(pSfr->port2 & PAN_LIMIT_SW);
    for ( i = 0; i < 45450; i++);   // ~100mSec, timer ticks do not have to be running
    switch_state2 = (int)(pSfr->port2 & PAN_LIMIT_SW);

    if ( switch_state1 != switch_state2 )
        return 0;


    return (switch_state1 ? 0 : 1);
}
