# Configured specifically for my situation. Should work for all Linux and OSX, I think.
UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Linux)
	CC := g++
	CFLAGS := -g
endif
ifeq ($(UNAME_S),Darwin)
	CC := clang++
	CFLAGS := -g -std=c++11 -stdlib=libc++
endif

SRCDIR := src
BUILDDIR := build
TARGET := bin/runner
 
SRCEXT := cpp
SOURCES := $(shell find $(SRCDIR) -type f -name *.$(SRCEXT))
OBJECTS := $(patsubst $(SRCDIR)/%,$(BUILDDIR)/%,$(SOURCES:.$(SRCEXT)=.o))
OBJECTS_TEST := $(filter-out %/main.o, $(OBJECTS))
LIB := -L lib
INC := -I include 
DEP = $(OBJECTS:%.o=%.d)

$(TARGET): $(OBJECTS)
	@echo " Linking..."
	@echo " $(CC) $^ -o $(TARGET) $(LIB)"; $(CC) $^ -o $(TARGET) $(LIB)

$(BUILDDIR)/%.o: $(SRCDIR)/%.$(SRCEXT)
	@mkdir -p $(BUILDDIR)
	@echo " $(CC) $(CFLAGS) -MMD $(INC) -c -o $@ $<"; $(CC) $(CFLAGS) -MMD $(INC) -c -o $@ $<

-include $(DEP)

clean:
	@echo " Cleaning..."; 
	@echo " $(RM) -r $(BUILDDIR) $(TARGET) bin/tuning"; $(RM) -r $(BUILDDIR) $(TARGET) bin/tuning

# Tests
tuning: $(OBJECTS_TEST)
	$(CC) $(CFLAGS) test/tuning.cpp $^ $(INC) -I src $(LIB) -o bin/tuning

interp: $(OBJECTS_TEST)
	$(CC) $(CFLAGS) test/test_interpolate.cpp $^ $(INC) -I src $(LIB) -o bin/test_interpolate

.PHONY: clean

