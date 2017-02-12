TARGET = mularsky
CC     = gcc
CFLAGS = -Wall -pedantic -std=c99

all: $(TARGET)

# VPATH = src src/pressure/
SRC = src/main.c \
	src/pressure/bme280.c \
	src/pressure/bme280_support.c

OBJS = $(SRC:.c=.o)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS)  -o $@ $(OBJS)


clean:
	find ./ -type f -name \*.o | xargs rm
	rm $(TARGET)
