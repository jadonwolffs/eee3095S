.RECIPEPREFIX +=
CC = gcc
CFLAGS = -Wall -lm -lrt -lwiringPi

PROG = bin/*
OBJS = obj/*

bin/Clock:
    mkdir -p bin obj
    $(CC) $(CFLAGS) -c src/BinClock.c -o obj/BinClock
    $(CC) $(CFLAGS) -c src/CurrentTime.c -o obj/CurrentTime
    $(CC) $(CFLAGS) obj/BinClock obj/CurrentTime -o bin/Clock

default:bin/Clock

run: bin/Clock
    sudo ./bin/Clock

clean:
    rm -f $(PROG) $(OBJS)

gpio:
    gpio readall
    @echo && echo =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    @echo && gpio i2cdetect | grep 6
    @echo && echo =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#    gpio load i2c

edit:
    vim src/BinClock.c

header:
    vim src/BinClock.h

debug: gpio clean run
