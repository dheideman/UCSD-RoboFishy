# project name 
# change to match your main c file
TARGET = RangeCal

TOUCH		 := $(shell touch *)
CC	:= g++
LINKER	 := g++ -o
CFLAGS	:= -c -Wall -g -I/usr/include/python2.7 `pkg-config --cflags opencv`
LFLAGS	:= -lwiringPi -lpython2.7 -I/usr/include/python2.7 -lwiringPiPca9685 -lpthread

# Local Source Variables
SOURCES	 := $(wildcard *.cpp)
INCLUDES := $(wildcard *.h)
OBJECTS	 := $(SOURCES:$%.cpp=$%.o)

# Libraries (for OpenCV) and RM
LIBS	= `pkg-config --libs opencv`
RM	 := rm -f

INSTALL_DIR = /usr/bin/

# linking Objects
$(TARGET): $(OBJECTS)
	@$(LINKER) $(@) $(OBJECTS) $(LFLAGS) $(LIBS)
	@echo
	@echo "Linking Complete"

echo:
	@echo $(INCLUDES)
	@echo $(SOURCES)
	@echo $(OBJECTS)

# compiling command
$(OBJECTS): %.o : %.cpp $(INCLUDES)
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
