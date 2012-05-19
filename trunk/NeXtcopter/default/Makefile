###############################################################################
# Makefile for the project NeXtcopter
###############################################################################

## General Flags
PROJECT = NeXtcopter
MCU = atmega168p
TARGET = NeXtcopter.elf
CC = avr-gcc

CPP = avr-g++

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
CFLAGS += -Wall -gdwarf-2 -std=gnu99       -DF_CPU=8000000UL -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d 

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS +=  -Wl,-Map=NeXtcopter.map


## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom -R .fuse -R .lock -R .signature

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings


## Objects that must be built in order to link
OBJECTS = adc.o eeprom.o FC_main.o gyros.o init.o isr.o motors.o pots.o rc.o 

## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
all: $(TARGET) NeXtcopter.hex NeXtcopter.eep NeXtcopter.lss size

## Compile
adc.o: ../src/adc.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

eeprom.o: ../src/eeprom.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

FC_main.o: ../src/FC_main.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

gyros.o: ../src/gyros.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

init.o: ../src/init.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

isr.o: ../src/isr.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

motors.o: ../src/motors.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

pots.o: ../src/pots.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

rc.o: ../src/rc.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

##Link
$(TARGET): $(OBJECTS)
	 $(CC) $(LDFLAGS) $(OBJECTS) $(LINKONLYOBJECTS) $(LIBDIRS) $(LIBS) -o $(TARGET)

%.hex: $(TARGET)
	avr-objcopy -O ihex $(HEX_FLASH_FLAGS)  $< $@

%.eep: $(TARGET)
	-avr-objcopy $(HEX_EEPROM_FLAGS) -O ihex $< $@ || exit 0

%.lss: $(TARGET)
	avr-objdump -h -S $< > $@

size: ${TARGET}
	@echo
	@avr-size -C --mcu=${MCU} ${TARGET}

## Clean target
.PHONY: clean
clean:
	-rm -rf $(OBJECTS) NeXtcopter.elf dep/* NeXtcopter.hex NeXtcopter.eep NeXtcopter.lss NeXtcopter.map


## Other dependencies
-include $(shell mkdir dep 2>/dev/null) $(wildcard dep/*)

