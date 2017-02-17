TARGET = mularsky
CC     = gcc
CFLAGS = -Wall -pedantic -std=c99 -I./src/pressure/ -I./src/ -O
LIBS   =


all: $(TARGET)

VPATH = src src/pressure/
SRC = src/main.c \
	src/pressure/bme280.c \
	src/pressure/bme280_support.c

OBJS = $(SRC:.c=.o)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $(LIBS)  -o $@ $(OBJS)


clean:
	find ./ -type f -name \*.o | xargs rm
	rm $(TARGET)
