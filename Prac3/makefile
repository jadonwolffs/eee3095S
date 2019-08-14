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
    rm $(PROG) $(OBJS)

gpio:
    gpio load i2c
    gpio readall
    gpio i2cdetect