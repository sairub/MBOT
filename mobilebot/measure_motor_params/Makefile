# this target will be compiled
TARGET = ../bin/measure_motor_params

CC		    := gcc
LINKER		:= gcc -o
CFLAGS		:= -c -Wall -g `pkg-config --cflags lcm`
LFLAGS		:= -lm -lrt -lpthread -l:librobotcontrol.so.1 -L `pkg-config --libs lcm`

SOURCES		:= $(wildcard *.c)
SOURCES		:= $(SOURCES) $(wildcard ../common/*.c)
INCLUDES	:= $(wildcard *.h)
INCLUDES	:= $(INCLUDES) $(wildcard ../common/*.h)
OBJECTS		:= $(SOURCES:$%.c=$%.o)
LCMTYPES    := $(shell find ../lcmtypes/ -name '*.lcm')                        
LCMOBJS     := $(LCMTYPES:$%.lcm=$%.o) 

prefix		:= /usr/local
RM		    := rm -f
INSTALL		:= install -m 4755
INSTALLDIR	:= install -d -m 755 

LINK		:= ln -s -f
LINKDIR	    := /etc/librobotcontrol
LINKNAME	:= link_to_startup_program


# linking Objects
$(TARGET): $(OBJECTS)
	@$(LINKER) $@ $(OBJECTS) $(LCMOBJS) $(LFLAGS)


# compiling command
$(OBJECTS): %.o : %.c $(INCLUDES)
	@$(CC) $(CFLAGS) -c $< -o $(@)
	@echo "Compiled: "$<

#  make all is the same as make
all: 
	$(TARGET)

debug:
	$(MAKE) $(MAKEFILE) DEBUGFLAG="-g -D DEBUG"
	@echo " "
	@echo "$(TARGET) Make Debug Complete"
	@echo " "

install:
	@$(MAKE) --no-print-directory
	@$(INSTALLDIR) $(DESTDIR)$(prefix)/bin
	@$(INSTALL) $(TARGET) $(DESTDIR)$(prefix)/bin
	@echo "$(TARGET) Install Complete"

clean:
	@$(RM) $(OBJECTS)
	@$(RM) $(LCMOBJS)
	@$(RM) $(IGNORE)
	@$(RM) $(IGNORE:=.o)
	@$(RM) $(TARGET)
	@echo "$(TARGET) Clean Complete"

uninstall:
	@$(RM) $(DESTDIR)$(prefix)/bin/$(TARGET)
	@echo "$(TARGET) Uninstall Complete"

runonboot:
	@$(MAKE) install --no-print-directory
	@$(LINK) $(DESTDIR)$(prefix)/bin/$(TARGET) $(LINKDIR)/$(LINKNAME)
	@echo "$(TARGET) Set to Run on Boot"

