/* 
 * ptctrl.c
 *
 *  Pan-tilt system controller program.
 *
 */
#define     __STDC_WANT_LIB_EXT1__  1   // Safe library function calls

#include    <stdint.h>
#include    <stdlib.h>
#include    <string.h>
#include    <stdio.h>
#include    <stdarg.h>
#include    <unistd.h>

#include    "v25.h"
#include    "pt-driver.h"
#include    "uart.h"

/****************************************************************************
  Module definitions
****************************************************************************/
#define     VERSION                 "v1.0"
#define     USAGE                   "Usage: ptctrl [-V | -h | -r <baud>]"
#define     HELP                    USAGE                                                       \
                                    "\n"                                                        \
                                    "-V  version info\n"                                        \
                                    "-h  help\n"                                                \
                                    "-r  set mode to remote with baud rate\n"                   \
                                    "    valid: '9600', '19200', '38400', '57600', '115200'\n"  \
                                    "    default without -r is 9600\n"

// UART command line processing
#define     CLI_BUFFER      80
#define     CR              0x0d    // Carriage return
#define     LF              0x0a    // Line feed
#define     BELL            0x07    // Bell
#define     BS              0x08    // Back space
#define     SPACE           0x20    // Space
#define     MAX_TOKENS      7       // Max number of command line tokens
#define     PRINT_BUFFER    80

// CLI definitions
#define     CMD_DELIM       " \t"   // Command line white-space delimiters
#define     ERR_PROMPT      "!"
#define     PROMPT          ">"
#define     SYNTAX_ERR      "Syntax error.\n"
#define     PARAM_ERR       "Parameter error.\n"
#define     HELP_TEXT       "\n\
  help                          - Help text\n\
  home                          - Home position\n\
  get pos                       - Print motor positions\n\
  get a2d <ch>                  - Read A-to-D channel\n\
  get microsteps                - Get micro-steps setting\n\
  move <motor> <+/-rate> <step> - Move relative steps\n\
  sync <+/-p-rate> <p-step>\n\
       <+/-t-rate> <t-step>     - Synchronized start motors\n\
  pan  <+/-rate> <step>         - Pan system\n\
  tilt <+/-rate> <step>         - Tilt system\n\
  wait all|pan|tilt             - Wait for motor to stop\n\
  stop                          - Stop all motors\n\
  mode interactive|remote       - Control mode\n\
  version                       - Show version\n"

/****************************************************************************
  Types
****************************************************************************/

/****************************************************************************
  Function prototypes
****************************************************************************/
int  process_cli(char*);
void print_str(const char *);
int  vprint_str(char *, ...);
int  str2int(char *, int *);
int  sign(int);

/****************************************************************************
  Globals
****************************************************************************/
int          interactive = 1;       // Print prompt if interactive mode

/* ----------------------------------------------------------------------------
 * main() control functions
 *
 */
int main(int argc, char* argv[])
{
    static char     commandLine[CLI_BUFFER] = {0};
    static int      nCliIndex;
    static uint8_t  inChar;

    int     c;
    int     cli_result;
    baud_t  baud_rate = BAUD_9600;  // NEC V25 Flashlite default baud rate

    /* Parse command line variables
     */
    while ( ( c = getopt(argc, argv, ":Vhr:")) != -1 )
    {
        switch ( c )
        {
            case 'V':
                printf("ptctrl.exe %s %s %s\n", VERSION, __DATE__, __TIME__);
                return 0;

            case 'h':
                printf("%s\n", HELP);
                return 0;

            case 'r':
                interactive = 0;
                if ( strcmp(optarg, "9600") == 0 )
                    baud_rate = BAUD_9600;
                else if ( strcmp(optarg, "19200") == 0 )
                    baud_rate = BAUD_19200;
                else if ( strcmp(optarg, "38400") == 0 )
                    baud_rate = BAUD_38400;
                else if ( strcmp(optarg, "57600") == 0 )
                    baud_rate = BAUD_57600;
                else if ( strcmp(optarg, "115200") == 0 )
                    baud_rate = BAUD_115200;
                else
                {
                    printf("Valid baud rates: '9600', '19200', '38400', '57600', '115200'\n");
                    return 1;
                }
                break;

            case ':':
                if ( optopt == 'r')
                    printf( "'-%c' without baud parameter\n", optopt);
                return 1;

            case '?':
                printf("%s\n", USAGE);
                return 1;
        }
    }

    /* Initialize command line buffer
     */
    memset(commandLine, 0, CLI_BUFFER);
    nCliIndex = 0;

    /* IO, timer, UART, and interrupt initialization
     */
    io_init();

    if ( !interactive && sys_limit_switch() )
    {
        printf("'-r' switch specified, but limit switch pressed. Exiting.\n\n");
        return 0;
    }

    timer_isr_init();
    uart_init(baud_rate);

    if ( interactive )
        print_str("Homing...\n");

    sys_home();

    /* Clear console and output banner message
     */
    if ( interactive )
        {
            vprint_str("%c[2J", 27);
            vprint_str("Pan-Tilt controller %s %s\n", __DATE__, __TIME__);
        }

    print_str(PROMPT);

    /* Loop forever and scan for commands from console or host
     */
    while ( 1 )
    {
        if ( uart_ischar() )
        {
            inChar = uart_getchr();
            switch ( inChar )
            {
                case CR:
                    uart_putchr(inChar);                        // output a CR-LF even in 'remote' mode
                    uart_putchr(LF);

                    cli_result = process_cli(commandLine);      // -- process command --
                    if ( cli_result == 0 )
                    {
                        print_str(PROMPT);
                    }
                    else if ( cli_result == -1 )
                    {
                        if ( interactive )
                        {
                            print_str(PARAM_ERR);
                            print_str(PROMPT);
                        }
                        else
                        {
                            print_str(ERR_PROMPT);
                        }
                    }
                    else if ( cli_result == -2 )
                    {
                        if ( interactive )
                        {
                            print_str(SYNTAX_ERR);
                            print_str(PROMPT);
                        }
                        else
                        {
                            print_str(ERR_PROMPT);
                        }
                    }

                    memset(commandLine, 0, CLI_BUFFER);         // reinitialize command line buffer for next command
                    nCliIndex = 0;
                    break;

                default:
                    {
                        if ( nCliIndex < CLI_BUFFER )
                        {
                            if ( inChar != BS )                 // is character a back-space?
                                commandLine[nCliIndex++] = inChar;  // no, then store in command line buffer
                            else if ( nCliIndex > 0 )           // yes, it is a back-space, but do we have characters to remove?
                            {
                                nCliIndex--;                    // yes, remove the character
                                commandLine[nCliIndex] = 0;
                                if ( interactive )
                                {
                                    uart_putchr(BS);
                                    uart_putchr(SPACE);
                                }
                            }
                            else
                                inChar = 0;                     // no, so do nothing
                        }
                        else
                            inChar = BELL;

                        if ( interactive )
                            uart_putchr(inChar);                // echo character to console
                    }
            }
        } /* UART character input */
    } /* Endless while loop */

    return 0;
}

/* ----------------------------------------------------------------------------
 * process_cli()
 *
 * Process the command line text and execute appropriate action
 * Return '-1' on parameter error, '-2' on syntax error, otherwise '0'
 *
 * param:   Command line string to parse
 * return:  '0' no error, '-1' parameter error, '-2' syntax error
 *
 */
int process_cli(char *commandLine)
{
    char       *tokens[MAX_TOKENS] = {0, 0, 0, 0, 0, 0, 0};
    char       *token;
    char       *tempCli;
    int         numTokens;

    int         i, intTemp;

    int         motor, rate, steps;
    int         s_rate, s_steps;
    direction_t dir, s_dir;

    // Separate command line into tokens
    tempCli = commandLine;
    for ( numTokens = 0; numTokens < MAX_TOKENS; numTokens++, tempCli = NULL)
    {
        token = strtok(tempCli, CMD_DELIM);
        if ( token == NULL )
            break;
        tokens[numTokens] = token;
    }

    // If nothing found then this is an empty line, just exit
    if ( numTokens == 0 )
        return 0;

    /* Parse and execute commands starting with a
     * simple CLI 'help' printout
     */
    if ( strcmp(tokens[0], "help") == 0 )
    {
        if ( interactive )
            print_str(HELP_TEXT);

        return 0;
    }

    /* Move to home position
     */
    else if ( strcmp(tokens[0], "home") == 0 )
    {
        sys_home();
        return 0;
    }

    /* Get system state:
     * - Motor positions in steps relative to home
     * - A-to-D channel conversion
     * - A4988 driver micro-step setting
     */
    else if ( strcmp(tokens[0], "get") == 0 )
    {
        if ( strcmp(tokens[1], "pos") == 0 )
        {
            if ( interactive )
            {
                print_str("motor curr_pos\n");
                vprint_str("%5d %8d\n", MOTOR_PAN, motor_get_position(MOTOR_PAN));
                vprint_str("%5d %8d\n", MOTOR_TILT, motor_get_position(MOTOR_TILT));
            }
            else
            {
                vprint_str("%d,%d\n", MOTOR_PAN, motor_get_position(MOTOR_PAN));
                vprint_str("%d,%d\n", MOTOR_TILT, motor_get_position(MOTOR_TILT));
            }
            return 0;
        }
        else if ( strcmp(tokens[1], "a2d") == 0 &&
                  numTokens == 3 )
        {
            intTemp = atoi(tokens[2]);
            if ( intTemp < 0 || intTemp > 7 )
                return -1;

            vprint_str("%u\n", sys_a2d_read((uint8_t)intTemp));
            return 0;
        }
        else if ( strcmp(tokens[1], "microsteps") == 0 )
        {
            vprint_str("%d\n", sys_get_microsteps());
            return 0;
        }
    }

    /* Get movement command.
     * Motor start moving as soon as command is interpreted.
     * Range checks are done by the driver and errors will result
     * in 'Parameter error.'
     * Command assumes four parameters:
     *   move <motor> <rate> <step>
     */
    else if ( strcmp(tokens[0], "move") == 0 &&
              numTokens == 4 )
    {
        if ( !str2int(tokens[1], &motor)   ||
             !str2int(tokens[2], &rate)    ||
             !str2int(tokens[3], &steps) )
        {
            return -1;
        }

        dir = sign(rate);

        return motor_run((motor_t)motor, dir, abs(rate), steps);
    }

    /* Synchronized start of both pan and tilt
     * Command assumes six parameters:
     *  sync <p-rate> <p-step> <t-rate> <t-step>
     */
    else if ( strcmp(tokens[0], "sync") == 0 &&
              numTokens == 5 )
    {
        if ( !str2int(tokens[1], &rate)    ||
             !str2int(tokens[2], &steps)   ||
             !str2int(tokens[3], &s_rate)  ||
             !str2int(tokens[4], &s_steps) )
        {
            return -1;
        }

        dir = sign(rate);
        s_dir = sign(s_rate);

        intTemp = motor_run(MOTOR_PAN, dir, abs(rate), steps);
        if ( intTemp == 0)
        {
            intTemp = motor_run(MOTOR_TILT, s_dir, abs(s_rate), s_steps);
            if ( intTemp != 0 )
            {
                motor_stop(MOTOR_PAN);  // Stop pan motor if cannot tilt
            }
        }

        return intTemp;
    }

    /* Pan the system, moves both motors.
     * Range checks are done by the driver and errors will result
     * in 'Parameter error.'
     * Command assumes four parameters:
     *   pan <rate> <step>
     */
    else if ( strcmp(tokens[0], "pan") == 0 &&
              numTokens == 3 )
    {
        if ( !str2int(tokens[1], &rate)    ||
             !str2int(tokens[2], &steps) )
        {
            return -1;
        }

        dir = sign(rate);

        return sys_pan(dir, abs(rate), steps);
    }

    /* Tilt the system, moves tilt motor only.
     * Range checks are done by the driver and errors will result
     * in 'Parameter error.'
     * Command assumes four parameters:
     *   tilt <rate> <step>
     */
    else if ( strcmp(tokens[0], "tilt") == 0 &&
              numTokens == 3 )
    {
        if ( !str2int(tokens[1], &rate)    ||
             !str2int(tokens[2], &steps) )
        {
            return -1;
        }

        dir = sign(rate);

        return sys_tilt(dir, abs(rate), steps);
    }

    /* Wait for specified motors to stop
     */
    else if ( strcmp(tokens[0], "wait") == 0 )
    {
        if ( strcmp(tokens[1], "all") == 0 )
        {
            sys_wait_all_stop();
            return 0;
        }
        else if ( strcmp(tokens[1], "pan") == 0 )
        {
            sys_wait_stop(MOTOR_PAN);
            return 0;
        }
        else if ( strcmp(tokens[1], "tilt") == 0 )
        {
            sys_wait_stop(MOTOR_TILT);
            return 0;
        }
    }

    /* Switch between interactive mode and remote control mode.
     * Use 'remote' control for sending commands and receiving
     * responses through serial connection.
     */
    else if ( strcmp(tokens[0], "mode") == 0 )
    {
        if ( strcmp(tokens[1], "interactive") == 0 )
        {
            interactive = 1;
            return 0;
        }
        else if ( strcmp(tokens[1], "remote") == 0 )
        {
            interactive = 0;
            return 0;
        }
    }

    /* Display firmware version
     */
    else if ( strcmp(tokens[0], "version") == 0 )
    {
        vprint_str("%s %s %s\n", VERSION, __DATE__, __TIME__);
        return 0;
    }

    /* Stop all motors by setting the motor's 'dir' to 0
     * Motors will stop at the next invocation of the timer0 interrupt
     */
    else if ( strcmp(tokens[0], "stop") == 0 )
    {
        motor_stop(MOTOR_PAN);
        motor_stop(MOTOR_TILT);
        return 0;
    }

    /* Fall through to signal
     * error in command line syntax
     */
    return -2;
}

/* ----------------------------------------------------------------------------
 * print_str()
 *
 * Send a NULL-terminated string down the UART Tx
 *
 * param:   String to send/print
 * return:  nothing
 *
 */
void print_str(const char *s)
{

  while (*s)
    {
      if (*s == '\n')
          uart_putchr('\r');

      uart_putchr(*s++);
    }
}

/* ----------------------------------------------------------------------------
 * vprint_str()
 *
 * Print a formatter string
 *
 * param:   String format and parameters to send/print
 * return:  Number of characters printed
 *
 */
int vprint_str(char *format, ...)
{
    char    text[PRINT_BUFFER] = {0};             // text buffer for printing messages
    va_list aptr;
    int     ret;

    va_start(aptr, format);
    ret = vsnprintf(text, PRINT_BUFFER, format, aptr);
    va_end(aptr);

    print_str(text);

    return ret;
}

/* ----------------------------------------------------------------------------
 * str2int()
 *
 * Convert a string to an integer, and also
 * return conversion status.
 *
 * param:   pointer to source string containing *only* digits and pointer to output integer
 * return:  '0' conversion error (string has non-digits in it), '-1' conversion ok
 *          Conversion result in 'integer' is only valid if function returns '-1'
 *
 */
int  str2int(char *string, int *integer)
{
    char   *end_character;

    *integer = (int) strtol(string, &end_character, 0);

    if ( *end_character != '\0' )
        return 0;

    return -1;
}

/* ----------------------------------------------------------------------------
 * sign()
 *
 * Return the sign of the input parameter or zero if it is zero.
 *
 * param:   Integer number
 * return:  '-1' for negative, '+1' for positive, or '0'
 *
 */
int sign(int x)
{
    return (x > 0) - (x < 0);
}
