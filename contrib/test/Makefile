# Includes
BASEDIR = ../..
IDIR =$(BASEDIR)
CFLAGS=-O0 -I$(IDIR)
CFLAGS+=-I/usr/lib/avr/include

OBJS=CU_test1.o
OBJS+=$(BASEDIR)/services/heating_controller/heating_ctrl.o

USER_OBJS=-lcunit

# All Target
all: CU_test1

# Tool invocations
CU_test1: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GCC C Linker'
	gcc  -g -o $@ $(OBJS) $(USER_OBJS) $(CFLAGS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

CU_test1.o : CU_test1.c
	$(CC) -g $(CFLAGS) -c $<


# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(EXECUTABLES) CU_tests
	-@echo ' '
	