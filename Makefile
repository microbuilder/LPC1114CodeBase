##########################################################################
# User configuration and firmware specific object files	
##########################################################################

# The target, flash and ram of the LPC1xxx microprocessor.
# Use for the target the value: LPC11xx, LPC13xx or LPC17xx
TARGET = LPC11xx
FLASH = 32K
SRAM = 8K

# For USB support the LPC1xxx reserves 384 bytes from the sram,
# if you don't want to use the USB features, just use 0 here.
SRAM_USB = 0

VPATH = 
OBJS = main.o

##########################################################################
# Project-specific files 
##########################################################################

VPATH += project
OBJS += commands.o eeprom.o

VPATH += project/commands
OBJS += cmd_chibi_addr.o cmd_chibi_tx.o cmd_deepsleep.o cmd_hello.o
OBJS += cmd_i2ceeprom_read.o cmd_i2ceeprom_write.o cmd_lm75b_gettemp.o
OBJS += cmd_sysinfo.o cmd_sd_dir.o

##########################################################################
# Optional driver files 
##########################################################################

# Chibi Light-Weight Wireless Stack (AT86RF212)
VPATH += drivers/chibi
OBJS += chb.o chb_buf.o chb_drvr.o chb_eeprom.o chb_spi.o

# 4K EEPROM
VPATH += drivers/eeprom/mcp24aa
OBJS += mcp24aa.o

# LM75B temperature sensor
VPATH += drivers/sensors/lm75b
OBJS += lm75b.o

# Bitmap LCD support (ST7565)
VPATH += drivers/lcd drivers/lcd/bitmap/st7565
OBJS += smallfonts.o ST7565.o

# ChaN FatFS and SD card support
VPATH += drivers/fatfs
OBJS += ff.o mmc.o

# PWM
VPATH += drivers/pwm
OBJS += pwm.o

# Motors
VPATH += drivers/motor/stepper
OBJS += stepper.o

##########################################################################
# Library files 
##########################################################################
VPATH += core core/adc core/cmd core/cpu core/gpio core/i2c core/pmu
VPATH += core/ssp core/systick core/timer16 core/timer32 core/uart
VPATH += core/libc core/wdt
OBJS += adc.o cpu.o cmd.o gpio.o i2c.o pmu.o ssp.o systick.o timer16.o
OBJS += timer32.o uart.o uart_buf.o stdio.o string.o wdt.o sysinit.o

##########################################################################
# GNU GCC compiler prefix and location
##########################################################################
CROSS_COMPILE = arm-none-eabi-
AS = $(CROSS_COMPILE)gcc
CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)gcc
SIZE = $(CROSS_COMPILE)size
OBJCOPY = $(CROSS_COMPILE)objcopy
OBJDUMP = $(CROSS_COMPILE)objdump
OUTFILE = firmware

##########################################################################
# GNU GCC compiler flags
##########################################################################
ROOT_PATH = .
INCLUDE_PATHS = -I$(ROOT_PATH) -I$(ROOT_PATH)/project

##########################################################################
# Startup files
##########################################################################

LD_PATH = lpc1xxx
LD_SCRIPT = $(LD_PATH)/linkscript.ld
LD_TEMP = $(LD_PATH)/memory.ld

ifeq (LPC11xx,$(TARGET))
  CORTEX_TYPE=m0
else
  CORTEX_TYPE=m3
endif

CPU_TYPE = cortex-$(CORTEX_TYPE)
VPATH += lpc1xxx
OBJS += $(TARGET)_handlers.o LPC1xxx_startup.o

##########################################################################
# Compiler settings, parameters and flags
##########################################################################
CFLAGS  = -c -Os $(INCLUDE_PATHS) -Wall -mthumb -ffunction-sections -fdata-sections -fmessage-length=0 -mcpu=$(CPU_TYPE) -DTARGET=$(TARGET) -fno-builtin
ASFLAGS = -c -Os $(INCLUDE_PATHS) -Wall -mthumb -ffunction-sections -fdata-sections -fmessage-length=0 -mcpu=$(CPU_TYPE) -D__ASSEMBLY__ -x assembler-with-cpp
LDFLAGS = -nostartfiles -mcpu=$(CPU_TYPE) -mthumb -Wl,--gc-sections
OCFLAGS = --strip-unneeded

all: firmware

%.o : %.c
	$(CC) $(CFLAGS) -o $@ $<

%.o : %.s
	$(AS) $(ASFLAGS) -o $@ $<

firmware: $(OBJS) $(SYS_OBJS)
	-@echo "MEMORY" > $(LD_TEMP)
	-@echo "{" >> $(LD_TEMP)
	-@echo "  flash(rx): ORIGIN = 0x00000000, LENGTH = $(FLASH)" >> $(LD_TEMP)
	-@echo "  sram(rwx): ORIGIN = 0x10000000+$(SRAM_USB), LENGTH = $(SRAM)-$(SRAM_USB)" >> $(LD_TEMP)
	-@echo "}" >> $(LD_TEMP)
	-@echo "INCLUDE $(LD_SCRIPT)" >> $(LD_TEMP)
	$(LD) $(LDFLAGS) -T $(LD_TEMP) -o $(OUTFILE).elf $(OBJS)
	-@echo ""
	$(SIZE) $(OUTFILE).elf
	-@echo ""
	$(OBJCOPY) $(OCFLAGS) -O binary $(OUTFILE).elf $(OUTFILE).bin
	$(OBJCOPY) $(OCFLAGS) -O ihex $(OUTFILE).elf $(OUTFILE).hex
  
clean:
	rm -f $(OBJS) $(LD_TEMP) $(OUTFILE).elf $(OUTFILE).bin $(OUTFILE).hex
