CPUFREQ=1000000
MCU=attiny44a

LOADER=avrdude
#AVRDUDE_PORT=/dev/cuaU1
AVRDUDE_PORT=/dev/ttyUSB0
AVRDUDE_PART=t44
AVRDUDE_HW=buspirate
#AVRDUDE_EXTRA=-xspifreq=7 -V

OPT=-Os

WARNFLAGS=-Wall -Wextra -Wmissing-prototypes
WARNFLAGS+=-Wno-unused-parameter -Wno-format-zero-length
WARNFLAGS+=-Werror

CFLAGS=-mmcu=${MCU} -DF_CPU=${CPUFREQ}UL ${WARNFLAGS} ${OPT} -std=gnu99
CFLAGS+=-funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS+=-g

LIBAVR_OBJS=

CC=avr-gcc
OBJCOPY=avr-objcopy

all: firmware.hex

firmware.elf: main.o ${LIBAVR_OBJS}
	${CC} ${CFLAGS} -o $@ main.o ${LIBAVR_OBJS} -L.

firmware.hex: firmware.elf
	${OBJCOPY} -j .text -j .data -O ihex firmware.elf $@

load: ${LOADER}

teensy: firmware.hex
	${SUDO} teensy_loader_cli -v -w -mmcu=${MCU} firmware.hex

avrdude: firmware.hex
	avrdude -P ${AVRDUDE_PORT} -p ${AVRDUDE_PART} -c ${AVRDUDE_HW} \
	    ${AVRDUDE_EXTRA} -e -U flash:w:firmware.hex

rfuse:
	avrdude -P ${AVRDUDE_PORT} -p ${AVRDUDE_PART} -c ${AVRDUDE_HW} \
	    ${AVRDUDE_EXTRA} -U lfuse:r:-:h -U hfuse:r:-:h -U efuse:r:-:h

#wfuse:
#	avrdude -P ${AVRDUDE_PORT} -p ${AVRDUDE_PART} -c ${AVRDUDE_HW} \
#	    ${AVRDUDE_EXTRA} \
#	    -U lfuse:w:0xff:m -U hfuse:w:0xd9:m -U efuse:w:0xf1:m

dfu: firmware.hex
	-${SUDO} dfu-programmer ${DFU_PART} erase
	${SUDO} dfu-programmer ${DFU_PART} flash firmware.hex
	${SUDO} dfu-programmer ${DFU_PART} reset

clean:
	rm -f *.elf *.o *.a *.core firmware.hex
