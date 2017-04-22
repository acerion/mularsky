TARGET = mularsky
TARGET_B = button
CC     = gcc
CFLAGS = -Wall -pedantic -std=c99 -I./src/pressure/ -I./src/ -O
LIBS   = -lpthread -lwiringPi


all: $(TARGET) $(TARGET_B)

VPATH = src src/pressure/
SRC = src/main.c \
	src/pressure/bme280.c \
	src/m_i2c.c \
	src/m_bme280.c \
	src/m_bno055.c
SRC_B = src/button.c


OBJS = $(SRC:.c=.o)
OBJS_B = $(SRC_B:.c=.o)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $(LIBS) -o $@ $(OBJS)

$(TARGET_B): $(OBJS_B)
	$(CC) $(CFLAGS) $(LIBS) -o $@ $(OBJS_B)

clean:
	find ./ -type f -name \*.o | xargs rm -f
	find ./ -type f -name \*~ | xargs rm -f
	rm -f $(TARGET) $(TARGET_B)
