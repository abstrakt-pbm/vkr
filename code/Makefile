TARGET = firmware

CC = arm-none-eabi-gcc
CXX = arm-none-eabi-g++
OBJCOPY = arm-none-eabi-objcopy

SOURCES_C = src/system_stm32f4xx.c
SOURCES_CPP = src/main.cpp
SOURCES_ASM = startup_stm32f411xe.s
INCLUDES = -Iinc

CPU = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
DEFINES = -DSTM32F411xE

CFLAGS = $(CPU) $(DEFINES) $(INCLUDES) -O2 -Wall -g3 -fdata-sections -ffunction-sections
CXXFLAGS = $(CFLAGS) -fno-exceptions -fno-rtti
LDFLAGS = $(CPU) -TSTM32F411CEUx.ld -Wl,--gc-sections

all: $(TARGET).bin

$(TARGET).elf: $(SOURCES_C) $(SOURCES_CPP) $(SOURCES_ASM)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $^ -o $@

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

clean:
	rm -f *.elf *.bin
