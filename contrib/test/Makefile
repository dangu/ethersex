# Includes
BASEDIR = ../..
IDIR =$(BASEDIR)
CFLAGS= -g -Wattributes -O0 -I$(IDIR) 
#CFLAGS+=-I/core/host
CFLAGS+=-I/usr/lib/avr/include
#CFLAGS+=-I/usr/include/glib-2.0
#CFLAGS+=-I/usr/lib/i386-linux-gnu/glib-2.0/include


OBJS=CU_test1.o
OBJS+=$(BASEDIR)/services/heating_controller/heating_ctrl.o
#OBJS+=$(BASEDIR)/core/host/printf.o

USER_OBJS=-lcunit

# All Target
all: CU_test1

# Tool invocations
CU_test1: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GCC C Linker'
	$(CC)  -o $@ $(OBJS) $(USER_OBJS) $(CFLAGS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

CU_test1.o : CU_test1.c
	$(CC) $(CFLAGS) -c $<


# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(EXECUTABLES) CU_tests
	-@echo ' '
	