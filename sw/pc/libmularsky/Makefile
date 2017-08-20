SHELL = /bin/sh
CC    = gcc
FLAGS        = -std=c99 -Iinclude
CFLAGS       = -fPIC -pedantic -Wall -Wextra -std=c99
LDFLAGS      = -shared
DEBUGFLAGS   = -O0 -D _DEBUG
RELEASEFLAGS = -O2 -D NDEBUG -combine -fwhole-program

TARGET  = libmularsky.so
HEADERS = $(shell echo include/*.h)


SRC = src/m_utils.c
OBJS = $(SRC:.c=.o)

PREFIX = $(DESTDIR)/usr/local
BINDIR = $(PREFIX)/bin

all: $(TARGET)



$(TARGET): $(OBJS)
	$(CC) $(FLAGS) $(CFLAGS) $(LDFLAGS) $(DEBUGFLAGS) -o $(TARGET) $(OBJS)



clean:
	rm -f $(OBJECTS) $(TARGET) src/*.o
