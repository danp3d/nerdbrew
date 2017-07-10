PROJECT_DIR = $(HOME)/projects/nerdbrew
AVR_GCC_VERSION := $(shell expr `avr-gcc -dumpversion | cut -f1` \>= 4.9)
ARDMK_DIR = $(HOME)/projects/Arduino-Makefile
ARDUINO_DIR = /Applications/Arduino.app/Contents/Java
ARDUINO_LIB_PATH = $(HOME)/Documents/Arduino/libraries
USER_LIB_PATH := $(realpath $(PROJECT_DIR)/libraries)
BOARD_TAG = pro
BOARD_SUB = 16MHzatmega328
AVR_TOOLS_DIR = /usr/local
AVRDUDE = /usr/local/bin/avrdude
CFLAGS_STD = -std=gnu11
CXXFLAGS_STD = -std=gnu++14
CXXFLAGS = -pedantic -Wall -Wextra
ifeq "$(AVR_GCC_VERSION)" "1"
	CXXFLAGS += -fdiagnostics-color
endif

CURRENT_DIR = $(shell basename $(CURDIR))
OBJDIR = $(PROJECT_DIR)/bin/$(CURRENT_DIR)/$(BOARD_TAG)

CTAGS_PATH = $(ARDUINO_DIR)/tools-builder/ctags/5.8-arduino11/ctags

include $(ARDMK_DIR)/Arduino.mk

.PHONY: ctags
ctags:
	$(CTAGS_PATH) -f tags.cpp $(shell find . -name "*.cpp" -o -name "*.h")
	$(CTAGS_PATH) -f tags.ino --langmap=c++:.ino $(shell find . -name "*.ino")
	cat tags.cpp tags.ino > tags
	sort tags -o tags
	rm -f tags.*

