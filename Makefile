CC = gcc
CFLAGS = -O3 -Wall -g -std=c11
# SCPATH = /usr/include/libxml2
# FANNARGS = -I fann/ -I fann/include/ fann/floatfann.c
# LINK = -lxml2 -lm -lfann
# OBJS = rgb_image.o parse.o detect.o shrink.o face.o
# HEADERS = parse.h rgb_image.h shrink.h face.h

all: calculation

detect: $(OBJS)
	$(CC) $(CFLAGS) -o calculation $(OBJS)

%.o: %.c $(HEADERS)
	$(CC) $(CFLAGS) -c -I -o $@ $<

clean:
	rm -rf *.o calculation
