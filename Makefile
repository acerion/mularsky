TARGET = mularsky
CC     = gcc
CFLAGS = -Wall -pedantic -std=c99 -I./src/pressure/ -I./src/ -O
LIBS   = -lpthread


all: $(TARGET)

VPATH = src src/pressure/
SRC = src/main.c \
	src/pressure/bme280.c \
	src/m_i2c.c \
	src/m_bme280.c \
	src/m_bno055.c


OBJS = $(SRC:.c=.o)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $(LIBS) -o $@ $(OBJS)


clean:
	find ./ -type f -name \*.o | xargs rm -f
	find ./ -type f -name \*~ | xargs rm -f
	rm -f $(TARGET)
