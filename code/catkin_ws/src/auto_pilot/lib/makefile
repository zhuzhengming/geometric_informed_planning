# Autopilot paths
SCRIPT_PATH := $(shell pwd)/../scripts
PYTHONPATH := ${PYTHONPATH}:$(SCRIPT_PATH)
LD_LIBRARY_PATH := ${LD_LIBRARY_PATH}:$(shell pwd)

# Find out how to link the python library 
PY3_CFLAGS  := $(shell python3-config --cflags) -fPIC 
PY3_LDFLAGS := $(shell python3-config --ldflags)

# General flags 
CC = g++
CPPFLAGS = -Wall
INC  = -I. -I./include
LDFLAGS = -L. -lautopilot 

all: shared test 
	@echo Built libautopilot.so and tested it  

# Build the shared object 
shared:
	@echo Building libautopilot.so...
	$(CC) $(INC) src/autopilot.cpp $(PY3_CFLAGS) $(PY3_LDFLAGS) -o libautopilot.so -shared 

# Build, test and remove generated executable 
test:
	@echo Building test_auto_pilot_api...
	$(CC) $(INC) src/test_auto_pilot_api.cpp -o test_auto_pilot_api $(LDFLAGS)
	./test_auto_pilot_api
	
.PHONY: clean 
clean: 
	rm test_auto_pilot_api libautopilot.so