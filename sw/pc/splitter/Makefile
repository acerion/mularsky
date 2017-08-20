TARGET = m_splitter
CC     = gcc
CFLAGS = -Wall -pedantic -std=c99 -I./src/ -I/home/acerion/include -L/home/acerion/lib
LIBS   = -lmularsky


all: $(TARGET)

VPATH = src
SRC = src/main.c
OBJS = $(SRC:.c=.o)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $(OBJS) $(LIBS)


clean:
	find ./ -type f -name \*.o | xargs rm -f
	find ./ -type f -name \*~ | xargs rm -f
	rm -f $(TARGET)
