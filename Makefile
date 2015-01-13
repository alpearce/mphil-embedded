CPPFLAGS=-I. -I../lib2
MCU=atmega644p
VPATH=../lib2
all: tap.hex

tap.elf: tap.o   spi2.o  usart2.o

%.o: %.c
	avr-gcc ${CPPFLAGS} -Os -mmcu=${MCU} -o $@ -c $^

%.elf: %.o
	avr-gcc -Os -mmcu=${MCU} -o $@ $^

%.hex: %.elf
	avr-objcopy -j .text -j .data -O ihex $^ $@

%.lst: %.elf
	avr-objdump -h -S $^ > $@

clean:
	rm -f *.o *.elf *.hex *.lst


program: tap.hex
	avrdude -p m644p -P /dev/ttyUSB0 -c avrusb500 -e -U flash:w:$^

fuses:
	avrdude -p m644p -P /dev/ttyUSB0 -c avrusb500 -e -U hfuse:w:0xDF:m 
	avrdude -p m644p -P /dev/ttyUSB0 -c avrusb500 -e -U lfuse:w:0xE2:m
