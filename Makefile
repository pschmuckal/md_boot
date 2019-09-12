CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
SIZE = arm-none-eabi-size

OUTNAME = sam_ba
SAM := SAMD51J18A
MCU := cortex-m4
OPT := s

SOURCEDIR = .
BUILDDIR = ./.build/$(kb)

LINK_DIR = ./bootloader

INC_PATHS = .
INC_PATHS += ..
INC_PATHS += ./bootloader/stub
INC_PATHS += ./packs/atmel/SAMD51_DFP/1.0.70/include
INC_PATHS += ./packs/arm/cmsis/5.0.1/CMSIS/Include
INC_PATHS += ./usb
ifdef kb
	kbpath = ./keyboards/massdrop/$(kb)
	INC_PATHS += $(kbpath)
endif

INC = $(INC_PATHS:%=-I%)

CFLAGS = -x c
CFLAGS += -mthumb
CFLAGS += -DNDEBUG
CFLAGS += -D__$(SAM)__
CFLAGS += -D__KB__=\"$(kb)\"
CFLAGS += -DMD_BOOTLOADER
CFLAGS += -DDEBUG_BOOT_TRACING
CFLAGS += $(INC)
CFLAGS += -O$(OPT)
CFLAGS += -ffunction-sections
CFLAGS += -mlong-calls
CFLAGS += -Wall
CFLAGS += -mcpu=$(MCU)
CFLAGS += -c
#CFLAGS += -S -Wa,-adhln
CFLAGS += -std=gnu99
CFLAGS += -MD
CFLAGS += -MP
CFLAGS += -MF "$(@:%.o=%.d)"
CFLAGS += -MT"$(@:%.o=%.d)"
CFLAGS += -MT"$(@:%.o=%.o)" 

LDFLAGS = -mthumb
LDFLAGS += -Wl,-Map=$(BUILDDIR)/$(OUTNAME).map
LDFLAGS += -Wl,--start-group
LDFLAGS += -lm
LDFLAGS += -Wl,--end-group
LDFLAGS += -L$(LINK_DIR)
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -mcpu=$(MCU)
LDFLAGS += -T$(LINK_DIR)/samd51j18a_flash.ld 

SOURCES = $(wildcard $(SOURCEDIR)/*.c)
SOURCES += $(wildcard $(SOURCEDIR)/bootloader/*.c)
SOURCES += $(wildcard $(SOURCEDIR)/bootloader/common/*.c)
SOURCES += $(wildcard $(SOURCEDIR)/usb/*.c)
SOURCES += $(wildcard $(SOURCEDIR)/$(kbpath)/*.c)

KB_SRC_X = $(SOURCEDIR)/$(kbpath)/matrix.c

SOURCESB = $(filter-out $(KB_SRC_X),$(SOURCES))

OBJECTS = $(patsubst $(SOURCEDIR)/%.c,$(BUILDDIR)/%.o,$(SOURCESB))

all: checks dir $(BUILDDIR)/$(OUTNAME).elf $(BUILDDIR)/$(OUTNAME).bin
	$(SIZE) $(BUILDDIR)/../massdrop_$(kb)_bootloader.elf

checks:
ifndef kb
	$(error You must set kb to the desired keyboard)
endif

dir:
	$(info Creating build directories)
	@mkdir -p $(BUILDDIR)
	@mkdir -p $(BUILDDIR)/bootloader
	@mkdir -p $(BUILDDIR)/bootloader/common
	@mkdir -p $(BUILDDIR)/usb
	@mkdir -p $(BUILDDIR)/$(kbpath)

$(BUILDDIR)/$(OUTNAME).bin: $(BUILDDIR)/$(OUTNAME).elf
	@echo -n Creating ${@}
	@$(OBJCOPY) -O binary $^ $@
	@mv $(BUILDDIR)/$(OUTNAME).elf $(BUILDDIR)/../massdrop_$(kb)_bootloader.elf
	@mv $(BUILDDIR)/$(OUTNAME).bin $(BUILDDIR)/../massdrop_$(kb)_bootloader.bin
	@mv $(BUILDDIR)/$(OUTNAME).map $(BUILDDIR)/../massdrop_$(kb)_bootloader.map
	@echo " Done!"

$(BUILDDIR)/$(OUTNAME).elf: $(OBJECTS)
	@echo -n Creating ${@}
	@$(CC) -o $@ $^ $(LDFLAGS)
	@echo " Done!"

$(OBJECTS): $(BUILDDIR)/%.o : $(SOURCEDIR)/%.c $(kbpath)/config.h
	$(info Compiling ${<}...)
	@$(CC) $(CFLAGS) $< -o $@

.PHONY: clean
clean:
	@echo -n "Cleaning..."
	@find $(BUILDDIR)/ -type f -name '*' -print0 | xargs -0 rm
	@echo " Done!"
