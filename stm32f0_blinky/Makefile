CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy

CFLAGS=-mcpu=cortex-m0 -mthumb -O0 -g -Wall -DSTM32F051xx \
       -I./ \
    
      

#LDFLAGS=-Tstm32g474re.ld -nostdlib -Wl,--gc-sections,-Map=blinky.map
LDFLAGS=-Tstm32f051r8t6.ld -nostdlib -Wl,--gc-sections,-Map=blinky.map -lc -lgcc -lnosys
SRCS = main.c \
       startup_stm32f051r8t6.c

OBJS = $(SRCS:.c=.o)

# Default target: build ELF + HEX + BIN
all: blinky.elf blinky.hex blinky.bin

# Link step
blinky.elf: $(OBJS)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

# Convert ELF -> HEX
blinky.hex: blinky.elf
	$(OBJCOPY) -O ihex $< $@

# Convert ELF -> BIN
blinky.bin: blinky.elf
	$(OBJCOPY) -O binary $< $@

# Compile C -> object
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# Clean up
clean:
	rm -f $(OBJS) blinky.elf blinky.hex blinky.bin blinky.map

