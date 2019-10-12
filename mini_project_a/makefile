.RECIPEPREFIX +=
CC = g++
CFLAGS = -Wall -lm -lrt -lwiringPi -lpthread

PROG = bin
OBJS = obj
primary = miniprojectA
secondary = CurrentTime
exe = project

/bin/$(primary):
    mkdir -p bin obj
    $(CC) $(CFLAGS) -c src/$(primary).c -o obj/$(primary)
    $(CC) $(CFLAGS) -c src/$(secondary).c -o obj/$(secondary)
    $(CC) $(CFLAGS) obj/$(primary) obj/$(secondary) -o bin/$(exe)

run: /bin/$(primary)
    ./$(PROG)/$(exe)

default = /bin/$(primary)

clean:
    rm -fr $(PROG)/* $(OBJS)/*
