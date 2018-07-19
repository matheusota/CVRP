CC = g++
TARGET = qroutes

CFLAGS   = -g -Wall

LINKER   = g++ -o


SRCDIR   = source
HEADDIR  = include
OBJDIR   = obj
BINDIR   = bin

SOURCES  := $(wildcard $(SRCDIR)/*.cpp)
INCLUDES := $(wildcard $(HEADDIR)/*.h)
OBJECTS  := $(SOURCES:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)


$(BINDIR)/$(TARGET): $(OBJECTS)
	$(LINKER) $@ $(LFLAGS) $(OBJECTS)
	@echo "Linking complete!"

$(OBJECTS): $(OBJDIR)/%.o : $(SRCDIR)/%.cpp
	@$(CC) $(CFLAGS) -c $< -Iinclude -o $@
	@echo "Compiled "$<" successfully!"

clean:
	rm -f $(OBJECTS)

