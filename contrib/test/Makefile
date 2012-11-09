# Includes
BASEDIR = ../..
IDIR =$(BASEDIR)
CFLAGS=-I$(IDIR)
CFLAGS+=-I/usr/lib/avr/include
#CFLAGS+=-I/usr/lib/gcc/avr/4.3.4/include
#CFLAGS+=-I/usr/lib/gcc/avr/4.3.4/include-fixed
#CFLAGS+=-I/usr/include
#CFLAGS+=-I/usr/local/include
#CFLAGS+=`pkg-config --cflags glib-2.0`

OBJS=CU_test1.o
#OBJS+=$(BASEDIR)/services/heating_controller/heating_ctrl.c
#OBJS+=$(BASEDIR)/core/host/printf.c

USER_OBJS=-lcunit

# All Target
all: CU_tests

# Tool invocations
CU_tests: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GCC C Linker'
	gcc  -o "test1" $(OBJS) $(USER_OBJS) $(CFLAGS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(EXECUTABLES) CU_tests
	-@echo ' '
	