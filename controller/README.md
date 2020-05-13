# Differential pan-tilt controller

This repository contains the main driver routines in ```pt-driver.c``` and an interrupt service for the driver in ```drv-isr.asm```. This driver is an improved version of what I used for my [robotics arm project](https://sites.google.com/site/eyalabraham/robotic-arm).  

The controller code is in module ```ptctrl.c```. The controller provides an interface to the pan-tilt system through a simple CLI that can be invoked from a simple serial console or in a 'remote' mode through the same serial link. The remote mode removes extra console output so that control commands can be issued by a serial-connected controller.

## System

```
 
  +-----+            +-------------+
  |     |            |             |
  | PC  |            | NEC V25 SBC +------------------------+
  |     +--< UART >--+             |                        |
  |     |            |             |       +------------+   |
  |     |            |             +-<SPI>-+ A/D MAX186 |   |
  |     |            |             |       +-+-----+----+   |
  +-----+            +---+-----+---+         |     |        |
                         |     |             |     |        |
                     +---+-+ +-+---+         |     |        |
                     |A4988| |A4988|         |  <sonar>     |
                     +---+-+ +-+---+         |              |
                         |     |       <Pan feedback>       |
                     +---+-+ +-+---+         |              |
                     | Pan | | Tilt|         |       <micro switch>
                     +---+-+ +-+---+         |              |
                         |     |             |              |
                     +---+-----+-------------+--------------+---+
                     |        Pan Tilt System                   |
                     +------------------------------------------+

```

## Hardware

Two TEAC 14769070-60 stepper motors, one for pan and one for tilt. Motor drivers use common micro-stepper driver [A4988](https://drive.google.com/open?id=1uaHy-gVYzntSpmbj-RvlSXfdfeH-IRd8). Controller CPU based on 10MHz [FlashLite NEC V25 SBC](https://lh5.googleusercontent.com/kj4iZ8OMkE5W92mf-GG1na3PRkUhlsCT2e-m8hr-VAq39MQ7tvtFeNA4bHNfKd3oUbItSMeTRnDGq1jqQkdHiuW65H7H5WfajjJDt6m5efU-0dvqxA=w1280) that is binary compatible with Intel 8088.  

The pan-tilt system has a servo-position potentiate on the pan axis and a micro switch on the tilt axis. These are used to center the pan axis and home the tilt axis.  

An optional sonar device (LV-MaxSonar-EZ) is connected as an analog distance provider to the A/D channel 1.

## Stepper motor measurements

|    Mode              | Voltage  |  Current    |
|----------------------|----------|-------------|
|  full step, one coil |  12v     |  150mA      |
|  full step, two coil |  12v     |  280mA      |
|  half step           |  12v     |  220-240mA  |

Stall current @ 12v:
- Single coil 145mA
- Two coils 300mA

### Coil arrangement

```
     ----------o White
    @                   72.5 ohm
     ----------o Brown
    @                   74.0 ohm
     ----------o Red

     ----------o Blue
    @                   73.3 ohm
     ----------o Brown
    @                   72.7 ohm
     ----------o Yellow
```

### Motor spec.

```
TEAC 14769070-90
Item No. : KR15008

Features :
a. Step angle = 0.9 deg/ step
b. Number of leads = 5 cables (bipolar)

Specifications :
a. Nominal Voltage = 5 Volt DC (1 Volt DC/ phase)
b. Current = 0.3 A/ phase
c. Torque = 0.5 Kg.cm
```

