CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy

CFLAGS = -mcpu=cortex-m0 -mthumb -O0 -Wall -I./Drivers/CMSIS/Core/Include -I./Drivers/CMSIS/Device/ST/STM32F0xx/Include -DSTM32F051x8
LDFLAGS = -T bootloader.ld -nostdlib -Wl,--gc-sections,-Map=boot.map

SRC =  src/main.c boot_startup.c bootloader1.c stm32f0xx_led_driver.c stm32f0xx_gpio_driver.c stm32f0xx_rcc_driver.c Drivers/CMSIS/Device/ST/STM32F0xx/Source/Templates/system_stm32f0xx.c
OBJ = $(SRC:.c=.o)

all: bootloader.elf bootloader.bin bootloader.hex

bootloader.elf: $(OBJ)
	$(CC) $(CFLAGS) $(OBJ) -o $@ $(LDFLAGS) -lgcc

bootloader.hex: bootloader.elf
	$(OBJCOPY) -O ihex $< $@

bootloader.bin: bootloader.elf
	$(OBJCOPY) -O binary $< $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f *.o *.elf *.bin *.hex *.map
