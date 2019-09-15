# Prac 4
Prac 4 deals with low quality audio playback over SPI. Consult the lab manual for more information.

## Folder Contents

* bin/
	- binary files when built
* doc/
	- ignored prac report
* obj/
	- object files when built
* src/
	- Prac4.cpp - main driver of the program
	- Prac4.h - header for the program
	- sound_16k_8bit.raw - raw audio file for use by the program

## Running the Prac
All running of the C code is done through ```make``` commands. 
- ```make``` will compile Prac4.c (the default make rule is to build /bin/Prac4)
- ```make run``` will run /bin/Prac4 and will call the build rule if not built yet
- ```make clean``` will clean the object and binary files that have been compiled
- ```make play``` will play the audio to the normal audio jack for comparison