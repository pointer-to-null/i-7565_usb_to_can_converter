CC=gcc
CFLAGS=-c -Wall
LDFLAGS= -lpthread

SOURCES_PATH=src/

SOURCES=src/main.c

OBJECTS=$(SOURCES:.c=.o)
EXECUTABLE=usb_to_can_converter

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(OBJECTS) -o $@ $(LDFLAGS)

.c.o:
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -f $(SOURCES_PATH)*.o $(EXECUTABLE)
