# project name 
# change to match your main c file
TARGET = PoCRangefinder

TOUCH		 := $(shell touch *)
CC	:= g++
LINKER	 := g++ -o
CFLAGS	:= -c -Wall -g -I/usr/include/python2.7 `pkg-config --cflags opencv`
LFLAGS	:= -lwiringPi -lpython2.7 -I/usr/include/python2.7 -lwiringPiPca9685 -lpthread

# Module Selection (add required modules to MODULES)
MOD_PATH := ../../Modules
MODULES := Core V4L2Control RangeFinder Camera

# Local Source Variables
SOURCES	 := $(wildcard *.cpp)
INCLUDES := $(wildcard *.h)
OBJECTS	 := $(SOURCES:$%.cpp=$%.o)

# Module Source Variables
MOD_SOURCES	 := $(foreach mod,$(MODULES),$(wildcard $(MOD_PATH)/$(mod)/*.cpp))
MOD_INCLUDES := $(foreach mod,$(MODULES),$(wildcard $(MOD_PATH)/$(mod)/*.h))
MOD_OBJECTS	 := $(MOD_SOURCES:$%.cpp=$%.o)

# Libraries (for OpenCV) and RM
LIBS	= `pkg-config --libs opencv`
RM	 := rm -f

INSTALL_DIR = /usr/bin/

# linking Objects
$(TARGET): $(OBJECTS) $(MOD_OBJECTS)
	@$(LINKER) $(@) $(OBJECTS) $(MOD_OBJECTS) $(LFLAGS) $(LIBS)
	@echo
	@echo "Linking Complete"

echo:
	echo $(OBJECTS)
	echo $(MOD_OBJECTS)

# compiling command
$(OBJECTS): %.o : %.cpp %.h
	@$(TOUCH) $(CC) $(CFLAGS) -c $< -o $(@)
	@echo "Compiled "$<" successfully!"
	
# module compiling command
$(MOD_OBJECTS): %.o : %.cpp %.h
	@$(TOUCH) $(CC) $(CFLAGS) -c $< -o $(@)
	@echo "Compiled "$<" successfully!"


# install to /usr/bin
$(phony all) : $(TARGET)
.PHONY: install

install: $(all)
	@$(MAKE)
	@install -m 0755 $(TARGET) $(INSTALL_DIR)
	@echo
	@echo "Project "$(TARGET)" installed to $(INSTALL_DIR)"
	@echo
	
clean:
	@$(RM) $(OBJECTS)
	@$(RM) $(TARGET)
	@echo "Cleanup complete!"
