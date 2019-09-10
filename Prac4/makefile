.RECIPEPREFIX +=
CC = g++
CFLAGS = -Wall -lm -lrt -lwiringPi -lpthread

PROG = bin/*
OBJS = obj/*
primary = Prac4

/bin/$(primary):
    mkdir -p bin obj
    $(CC) $(CFLAGS) -c src/$(primary).cpp -o obj/$(primary)
    $(CC) $(CFLAGS) obj/$(primary) -o bin/$(primary)

run: /bin/$(primary)
    sudo ./bin/$(primary)

default = /bin/$(primary)

clean:
    rm $(PROG) $(OBJS)
play:
    play -t raw -r 16k -e unsigned -b 8 -c 1 src/sound_16k_8bit.raw