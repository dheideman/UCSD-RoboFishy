# project name 
# change to match your main c file
TARGET = Core




TOUCH 	 := $(shell touch *)
CC	:= g++
LINKER   := g++ -o
CFLAGS	:= -c -Wall 
LFLAGS	:= -lpthread

SOURCES  := $(wildcard *.cpp)
INCLUDES := $(wildcard *.h)
OBJECTS  := $(SOURCES:$%.cpp=$%.o)
RM := rm -f

INSTALL_DIR = /usr/bin/

# compiling command
$(OBJECTS): %.o : %.cpp
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
