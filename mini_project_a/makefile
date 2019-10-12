.RECIPEPREFIX +=
CC = gcc
CFLAGS = -Wall -lm -lrt -lwiringPi

PROG = bin/*
OBJS = obj/*

default:
    mkdir -p bin obj
    $(CC) $(CFLAGS) -c src/miniprojectA.c -o obj/miniprojectA
    $(CC) $(CFLAGS) -c src/CurrentTime.c -o obj/CurrentTime
    $(CC) $(CFLAGS) obj/miniprojectA obj/CurrentTime -o bin/ADC

run:
    sudo ./bin/ADC

clean:
    rm $(PROG) $(OBJS)
