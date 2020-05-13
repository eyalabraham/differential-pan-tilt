#####################################################################################
#
#  This make file is for compiling the 
#  8-bit NEC v25 pan-tilt motor controller
#
#  Use:
#    clean      - clean environment
#    all        - build all outputs
#
#####################################################################################

#------------------------------------------------------------------------------------
# generate debug information wit WD.EXE
# change to 'yes' or 'no'
#------------------------------------------------------------------------------------
DEBUG = no

# remove existing implicit rule (specifically '%.o: %c')
.SUFFIXES:

#------------------------------------------------------------------------------------
# project directories
#------------------------------------------------------------------------------------
INCDIR = include
PRECOMP = ../ws.pch

VPATH = $(INCDIR)

#------------------------------------------------------------------------------------
# build utilities
#------------------------------------------------------------------------------------
ASM = wasm
CC = wcc
LIB = wlib
LINK = wlink

#------------------------------------------------------------------------------------
# tool options
#------------------------------------------------------------------------------------
ifeq ($(DEBUG),yes)
CCDBG = -d2
ASMDBG = -d1
LINKDBG = DEBUG LINES 
else
CCDBG = -d0
ASMDBG =
LINKDBG =
endif

# Options:
#  -s   no stack frame check
#  -oi  inline generation of certain functions
#  -zp1 pack strutures
#  -ms  small memory model
#
CCOPT = -0 -ms $(CCDBG) -zp1 -s -oi -i=/home/eyal/bin/watcom/h -i=$(INCDIR)
ASMOPT = -0 -ms $(ASMDBG)
TESTLINKCFG = LIBPATH /home/eyal/bin/watcom/lib286/dos \
              LIBPATH /home/eyal/bin/watcom/lib286     \
              FORMAT DOS                               \
              OPTION MAP=test                          \
              OPTION ELIMINATE

#------------------------------------------------------------------------------------
# some variables for the linker
# file names list
#------------------------------------------------------------------------------------
COM = ,
EMPTY =
SPC = $(EMPTY) $(EMPTY)

#------------------------------------------------------------------------------------
# new make patterns
#------------------------------------------------------------------------------------
%.o: %.c
	$(CC) $< $(CCOPT) -fo=$(notdir $@)

#------------------------------------------------------------------------------------
# build all targets
#------------------------------------------------------------------------------------
all: ptctrl test

#------------------------------------------------------------------------------------
# build interrupt service
#------------------------------------------------------------------------------------
drv-isr.o: drv-isr.asm
	$(ASM) $(ASMOPT) -fo=$@ $<

#------------------------------------------------------------------------------------
# build test.exe test program
#------------------------------------------------------------------------------------
test: test.exe

test.exe: test.o pt-driver.o drv-isr.o uart.o
	$(LINK) $(TESTLINKCFG) FILE $(subst $(SPC),$(COM),$(notdir $^)) NAME $@

#------------------------------------------------------------------------------------
# build ptctrl.exe controller program
#------------------------------------------------------------------------------------
ptctrl: ptctrl.exe

ptctrl.exe: ptctrl.o pt-driver.o drv-isr.o uart.o
	$(LINK) $(TESTLINKCFG) FILE $(subst $(SPC),$(COM),$(notdir $^)) NAME $@

#------------------------------------------------------------------------------------
# cleanup
#------------------------------------------------------------------------------------

.PHONY: clean

clean:
	rm -f *.exe
	rm -f *.o
	rm -f *.lst
	rm -f *.map
	rm -f *.lib
	rm -f *.bak
	rm -f *.cap
	rm -f *.err

