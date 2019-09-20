CC=/opt/avr8-gnu-toolchain-linux_x86_64/bin/avr-gcc
AVRPACK=/opt/avr8-packs
MCU=attiny1614
F_CPU=5000000
CFLAGS=-Os -ffunction-sections -Wl,--gc-sections -mrelax -g
SRCS=main.c i2c.c bme280.c i2c_slave.c
NAME=wifisensor
CARGS=-mmcu=$(MCU) -DF_CPU=$(F_CPU) $(CFLAGS) -B $(AVRPACK)/gcc/dev/attiny1614 -I $(AVRPACK)/include
UPDI=/opt/pyupdi/pyupdi.py -c /dev/ttyUSB0 -d tiny1614

all: $(NAME).hex

$(NAME).elf:
	$(CC) -o $(NAME).elf $(CARGS) $(SRCS)
	avr-objdump -d $(NAME).elf > $(NAME).lst
	avr-size $(NAME).elf

$(NAME).hex: $(NAME).elf
	avr-objcopy -j .text -j .data  -j .rodata -O ihex $(NAME).elf $(NAME).hex

clean:
	rm -f $(NAME).elf $(NAME).hex $(NAME).lst *~

upload:
	$(UPDI) -f $(NAME).hex

reset:
	$(UPDI) -v -r

