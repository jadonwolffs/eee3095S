# Mini Project A
## WLFJAD001 and ARNJAM004

This project is an enviroment logger which uses a Raspberry Pi 3B+ to interface with some enviroment readers to display and track various attributes of an environment.
It is built in C and C++ and makes use of WiringPi for interfacing with sensors and Blynk to transmit data to a mobile app.

To run the logger run `make clean run` from the home directory and to run the Blynk server navigate into the src directory and run `./build.sh raspberry` and then `./run.sh`
These commands need to be run from different terminals or at least different virtual terminals (through something like the `screen` program)