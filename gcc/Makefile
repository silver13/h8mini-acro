TARGET=h8mini
EXECUTABLE=h8mini.elf

CC=arm-none-eabi-gcc
LD=arm-none-eabi-gcc
AR=arm-none-eabi-ar
AS=arm-none-eabi-as
CP=arm-none-eabi-objcopy
OD=arm-none-eabi-objdump


DEFS = -DUSE_STDPERIPH_DRIVER
STARTUP = startup.s

MCU = cortex-m3
MCFLAGS = -mcpu=$(MCU) -mthumb -mlittle-endian -mthumb-interwork -ffast-math -fdata-sections -ffunction-sections

topdir = ..
INCLUDES = -I$(topdir)/H8mini-test/src/ \
	-I$(topdir)/CMSIS/ \
	-I$(topdir)/Peripherals/inc/

OPTIMIZE = -Os

CFLAGS = $(MCFLAGS)  $(OPTIMIZE)  $(DEFS) -I. -I./ $(INCLUDES)  -Wl,-T,flash.ld,-Map,output.map,--gc-sections -std=gnu99 
AFLAGS = $(MCFLAGS)

SRC = $(wildcard $(topdir)/H8mini_acro/src/*.c) \
	$(wildcard $(topdir)/H8mini_acro/src/*.cpp) \
	$(topdir)/CMSIS/system_gd32f1x0.c \
	core_cm3.c \
	$(topdir)/Peripherals/src/gd32f1x0_adc.c \
	$(topdir)/Peripherals/src/gd32f1x0_dma.c \
	$(topdir)/Peripherals/src/gd32f1x0_fmc.c \
	$(topdir)/Peripherals/src/gd32f1x0_gpio.c \
	$(topdir)/Peripherals/src/gd32f1x0_i2c.c \
	$(topdir)/Peripherals/src/gd32f1x0_rcc.c \
	$(topdir)/Peripherals/src/gd32f1x0_timer.c \
	$(topdir)/Peripherals/src/gd32f1x0_usart.c

OBJDIR = .
OBJ = $(SRC:%.c=$(OBJDIR)/%.o)
OBJ += Startup.o

all: $(TARGET)

$(TARGET).hex: $(EXECUTABLE)
	$(CP) -O ihex $^ $@
$(TARGET): $(EXECUTABLE)
	$(CP) -O binary $^ $@

$(EXECUTABLE): $(SRC) $(STARTUP)
	$(CC) $(CFLAGS) $^ -lm -lnosys -o $@
	

clean:
	rm -f Startup.lst $(TARGET) $(TARGET).lst $(OBJ) $(AUTOGEN) \
		$(TARGET).out $(TARGET).hex  $(TARGET).map \
		$(TARGET).dmp $(EXECUTABLE)
