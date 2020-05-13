;
;=====================================================
;  file: drv-isr.asm
;
; Large (memory model) stepper motor driver interrupt routine.
; NOTE:
;   - Assumes IDB is default 0ffh
;   - IO ports are configured for the application
;
; * Port-0 bit assignment
; *
; *  b7 b6 b5 b4 b3 b2 b1 b0
; *  |  |  |  |  |  |  |  |
; *  |  |  |  |  |  |  |  +--- 'o' Pan  step pulse
; *  |  |  |  |  |  |  +------ 'o' Pan  direction 0=CW, 1=CCW
; *  |  |  |  |  |  +--------- 'o' Tilt step pulse
; *  |  |  |  |  +------------ 'o' Tilt direction 0=CW, 1=CCW
; *  |  |  |  +--------------- 'i'
; *  |  |  +------------------ 'i'
; *  |  +--------------------- 'i'
; *  +------------------------ 'o' Interrupt time-base test point
; *
; * Port-2 bit assignment
; *
; *  b7 b6 b5 b4 b3 b2 b1 b0
; *  |  |  |  |  |  |  |  |
; *  |  |  |  |  |  |  |  +--- 'o' Pan  MS0  00=1/4, 10=1/8, 11=1/16
; *  |  |  |  |  |  |  +------ 'o'      MS1
; *  |  |  |  |  |  +--------- 'o' Tilt MS0  00=1/4, 10=1/8, 11=1/16
; *  |  |  |  |  +------------ 'o'      MS1
; *  |  |  |  +--------------- 'i'
; *  |  |  +------------------ 'i'
; *  |  +--------------------- 'i'
; *  +------------------------ 'i' A/D SSTB (A/D pin.16)
;
; April 2020 - Created
;
;=====================================================
;
                .8086
                .model      small
;
;----------------------------------------
; Externals
;----------------------------------------
;
                extern      _timer_ticks    : word  ; Timer tick uint16_t
                extern      _motors         : word  ; Array of motor_t structure
;
;----------------------------------------
; Motor data structure
;----------------------------------------
;
stepctrl_t      struct
;                                           C-type
rate            dw          ?           ; (int     )  Motor run rate steps per second [1..RATE_LIMIT]
isr_rate        dw          ?           ; (uint16_t)  ISR step pulse cycle
counter         dw          ?           ; (uint16_t)  Motor is stepped when counter == 0, decrement every time ISR runs
dir             dw          ?           ; (int     )  Direction, '-1' to CCW, '+1' to CW, '0' stop
steps           dw          ?           ; (int     )  Steps to move, if '-1' then move until stopped or limit reached
curr_pos        dw          ?           ; (int     )  Current position
limit_high      dw          ?           ; (int     )  High limit step count
limit_low       dw          ?           ; (int     )  Low limit step count
step_ctrl_bit   db          ?           ; (uint8_t )  Motor driver 'step' IO pin
dummy           db          ?           ; (uint8_t )  Alignment byte
;
stepctrl_t      ends
;
;----------------------------------------
; IO
;----------------------------------------
;
PORT0           equ         000h
PORT1           equ         008h
PORT2           equ         010h

MOTOR_COUNT     equ         2

HEARTBEAT_SET   equ         080h
HEARTBEAT_CLR   equ         07fh
;
;----------------------------------------
; End-Of-Interrupt macro
; Signals NEC V25 interrupt controller of an EOI
;----------------------------------------
;
FINT            macro
                db          0fh
                db          092h
                endm
;
;
                .code
                assume      ds:DGROUP, ss:nothing, es:nothing
;
;=====================================================
;  driver_isr_
;
;  Interrupt service routine.
;  Invoked every clock tick and sequences stepper motors.
;
;  Measured timing profile @ 5,000Hz interrupt clock rate:
;   200uSec repetition interval
;    20uSec minimum execution time (10%)
;    75uSec maximum execution time (38%)
;
;=====================================================
;
                public      driver_isr_
;
driver_isr_     proc        far
;
                push        ax
                push        cx
                push        si
                push        di
                push        es
;
                mov         di, 0f000h                              ; [ES:DI] pointer to NEC V25 SFR
                mov         es, di
                mov         di, 0ff00h
;
                or          byte ptr es:[di+PORT0], HEARTBEAT_SET   ; Set heartbeat line
;
                inc         _timer_ticks                            ; Increment timer ticks
;
                mov         si, offset _motors
                mov         cx, MOTOR_COUNT
motor_loop:
                cmp         word ptr [si+dir], 0                    ; When 'dir' is 0 then motor should not move
                jz          next_motor
;
                dec         word ptr [si+counter]                   ; When a motor's 'counter' is 0 it is time to move one step
                jnz         next_motor                              ; No move at this time slot
;
; Check position against limits 'limit_high' and 'limit_low'
; TODO This level of protection is not sufficient and requires actively
;      moving the motor back into safe limits. Active step back is
;      required after stopping the motor specifically for the tilt axis
;      that is linked to the pan movement.
;
                mov         ax, word ptr [si+curr_pos]              ; Get current position
                cmp         ax, word ptr [si+limit_high]            ; Compare position to high limit
                jg          over_high_limit
                cmp         ax, word ptr [si+limit_low]             ; Compare position to lower limit
                jl          under_low_limit
                jmp         check_step_count                        ; Within high/low limits, continue
over_high_limit:
                cmp         word ptr [si+dir], 0
                jl          check_step_count                        ; Allow motor to move if direction is CCW (dir = -1)
                mov         word ptr [si+dir], 0                    ;  otherwise stop motor
                jmp         next_motor
under_low_limit:
                cmp         word ptr [si+dir], 0
                jg          check_step_count                        ; Allow motor to move if direction is CW (dir = +1)
                mov         word ptr [si+dir], 0                    ;  otherwise stop motor
                jmp         next_motor
;
; If a 'step' count was specified then only move that number of steps until the value is 0.
; Decrement step count if 'steps' were specified (i.e. 'steps' != -1)
;
check_step_count:
                cmp         word ptr [si+steps], 0
                jl          move_motor                              ; Steps is '-1', no step limit, skip everything
                jg          have_step_count                         ; Steps specified, go update step count
                mov         word ptr [si+dir], 0                    ; Limit reached, stop motor
                jmp         next_motor
;
have_step_count:
                dec         word ptr [si+steps]                     ; Decrement step count
;
; Move the motor one step
; Direction is already set on the port pins outside of the ISR
;
move_motor:
                mov         al, byte ptr [si+step_ctrl_bit]         ; Get 'step' bit position
                or          byte ptr es:[di+PORT0], al              ; Set A4988 step bit (can also do XOR)
                not         al                                      ;  a little bit of delay
                and         byte ptr es:[di+PORT0], al              ; Clear A4988 step bit
;
; Update motor position
;
                mov         ax, word ptr [si+curr_pos]              ; Adjust current motor position
                add         ax, word ptr [si+dir]                   ;  as a relative step count
                mov         word ptr [si+curr_pos], ax
;
; Special handling on tilt position
; In a differential drive, panning steps cause tilt steps in the opposite direction
;
                cmp         cx, 2                                   ; Did we just update the pan motor?
                jne         not_pan_motor                           ; No, skip tilt step position adjustment
                mov         ax, word ptr [si+dir]                   ; Yes, get pan direction
                neg         ax                                      ;  and reverse it
                add         ax, word ptr [si+(sizeof stepctrl_t)+curr_pos] ; Adjust the tilt step position
                mov         word ptr [si+(sizeof stepctrl_t)+curr_pos], ax
;
; Refresh 'counter' based on movement calculated 'isr_rate'
;
not_pan_motor:
                mov         ax, word ptr [si+isr_rate]              ; Get step rate counter
                mov         word ptr [si+counter], ax               ;  and refresh counter
;
next_motor:
                add         si, sizeof stepctrl_t
                loop        motor_loop
;
                and         byte ptr es:[di+PORT0], HEARTBEAT_CLR   ; Clear heartbeat line
;
                pop         es
                pop         di
                pop         si
                pop         cx
                pop         ax
;
                FINT
                iret
;
driver_isr_     endp
;
                end
