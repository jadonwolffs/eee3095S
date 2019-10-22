.RECIPEPREFIX +=
# CC = g++
# CFLAGS = -Wall -lm -lrt -lwiringPi -lpthread -I .

# PROG = bin
# OBJS = obj
primary = project_a
# secondary = RTC
exe = project_a

# /bin/$(primary):
#     mkdir -p bin obj
#     $(CC) $(CFLAGS) -c src/$(primary).c -o obj/$(primary)
#     $(CC) $(CFLAGS) -c src/$(secondary).c -o obj/$(secondary)
#     $(CC) $(CFLAGS) obj/$(primary) obj/$(secondary) -o bin/$(exe)

# run: /bin/$(primary)
#     ./$(PROG)/$(exe)

# default = /bin/$(primary)

# clean:
#     rm -fr $(PROG)/* $(OBJS)/*

# blynk: 
#     cd src && ./build.sh raspberry

# blynkrun: blynk
#     screen -S blynk ./run.sh

default = run

run:
    python3 src/project_b.py

pull:
    git pull

clean:

