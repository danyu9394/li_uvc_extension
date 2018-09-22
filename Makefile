CROSS_COMPILE ?=

CC	:= $(CROSS_COMPILE)gcc
CFLAGS	?= -O2 -W -Wall
LDFLAGS	?=

%.o : %.c
	$(CC) $(CFLAGS) -c -o $@ $<

all: uvc_extension_extra

uvc_extension_extra: uvc_extension_extra.o
	$(CC) $(LDFLAGS) -o $@ $^

clean:
	-rm -f *.o
	-rm -f uvc_extension_extra

