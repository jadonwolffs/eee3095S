/*
 * Prac4.cpp
 * 
 * Originall written by Stefan Schröder and Dillion Heald
 * 
 * Adapted for EEE3096S 2019 by Keegan Crankshaw
 * 
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "Prac4.h"

using namespace std;

bool playing = true; // should be set false when paused
bool stopped = false; // If set to true, program should close
unsigned char buffer[2][BUFFER_SIZE][2];
int buffer_location = 0;
bool buffer_reading = 0; //using this to switch between column 0 and 1 - the first column
bool threadReady = false; //using this to finish writing the first column at the start of the song, before the column is played
long last_interrupt = -201;

// Configure your interrupts here.
// Don't forget to use debouncing.
void play_pause_isr(void){
    //Write your logic here
    long current_time = millis();
    if(current_time-last_interrupt>150) //debounce
    {
        if(playing){
            playing=false;
            printf("Pausing\n");
        }
        else{
          playing=true;
          printf("Playing\n");
        }
        last_interrupt=current_time;
    }
    
}

void stop_isr(void){
    // Write your logic here
    long current_time = millis();
    if(current_time-last_interrupt>200) //debounce
    {
        printf("Stopping\n");
        ctrlc(0);
        last_interrupt=current_time;
    }
    
}

/*
 * Setup Function. Called once 
 */
int setup_gpio(void){
    signal(SIGINT, ctrlc); //catch keyboard interupts
	signal(SIGABRT, catch_abort); //catch crashes to cleanup
    //Set up wiring Pi
    wiringPiSetup();
    //setting up the buttons
    //play pause
    printf("Adding interrupt to play\n");
    pinMode(PLAY_BUTTON,INPUT);
    pullUpDnControl(PLAY_BUTTON,PUD_UP);
    wiringPiISR(PLAY_BUTTON, INT_EDGE_RISING, play_pause_isr);
    //stop
    printf("Adding interrupt to stop\n");
    pinMode(STOP_BUTTON,INPUT);
    pullUpDnControl(STOP_BUTTON,PUD_UP);
    wiringPiISR(STOP_BUTTON, INT_EDGE_RISING, stop_isr);
    //setting up the SPI interface
    wiringPiSPISetup(SPI_CHAN,SPI_SPEED);
    return 0;
}

/* 
 * Thread that handles writing to SPI
 * 
 * You must pause writing to SPI if not playing is true (the player is paused)
 * When calling the function to write to SPI, take note of the last argument.
 * You don't need to use the returned value from the wiring pi SPI function
 * You need to use the buffer_location variable to check when you need to switch buffers
 */
void *playThread(void *threadargs){
    // If the thread isn't ready, don't do anything
    while(!threadReady)
        continue;
    
    //You need to only be playing if the stopped flag is false
    while(!stopped){
        //Code to suspend playing if paused
		while(!playing)
        {
            continue;
        }
        
        //Write the buffer out to SPI
        wiringPiSPIDataRW(SPI_CHAN, buffer[buffer_reading][buffer_location],2);
		
        //Do some maths to check if you need to toggle buffers
        buffer_location++;
        if(buffer_location >= BUFFER_SIZE) {
            buffer_location = 0;
            buffer_reading = !buffer_reading; 
        }
    }
    
    pthread_exit(NULL);
}

int main(){
    // Call the setup GPIO function
	if(setup_gpio()==-1){
        return 0;
    }
    
    /* Initialize thread with parameters
     * Set the play thread to have a 99 priority
     * Read https://docs.oracle.com/cd/E19455-01/806-5257/attrib-16/index.html
     */ 
    
    //Write your logic here
	pthread_attr_t tattr;
    pthread_t thread_id;
    int newprio = 99;
    sched_param param;
    
    pthread_attr_init (&tattr);
    pthread_attr_getschedparam (&tattr, &param); /* safe to get existing scheduling param */
    param.sched_priority = newprio; /* set the priority; others are unchanged */
    pthread_attr_setschedparam (&tattr, &param); /* setting the new scheduling param */
    pthread_create(&thread_id, &tattr, playThread, (void *)1); /* with new priority specified *
    
    /*
     * Read from the file, character by character
     * You need to perform two operations for each character read from the file
     * You will require bit shifting
     * 
     * buffer[buffer_writing][counter][0] needs to be set with the control bits
     * as well as the first few bits of audio
     * 
     * buffer[buffer_writing][counter][1] needs to be set with the last audio bits
     * 
     * Don't forget to check if you have pause set or not when writing to the buffer
     * 
     */
     
    // Open the file
    char character;
    FILE *filePointer;
    printf("%s\n", FILENAME);
    filePointer = fopen(FILENAME, "r"); // read mode

    if (filePointer == NULL) {
        perror("Error while opening the file.\n");
        exit(EXIT_FAILURE);
    }

    int counter = 0;
    int buffer_writing = 0;

    // Have a loop to read from the file
	 while((character = fgetc(filePointer)) != EOF){
        while(threadReady && buffer_writing==buffer_reading && counter==0){
            //waits in here after it has written to a side, and the thread is still reading from the other side
            continue;
        }
        //Set config bits for first 8 bit packet and OR with upper bits
        buffer[buffer_writing][counter][0] = 0x70 | character>>6; 
        //Set next 8 bit packet
        buffer[buffer_writing][counter][1] = character<<2; 

        counter++;
        if(counter >= BUFFER_SIZE+1){
            if(!threadReady){
                threadReady = true;
            }

            counter = 0;
            buffer_writing = (buffer_writing+1)%2;
        }

    }
     
    // Close the file
    fclose(filePointer);
    printf("Completed reading"); 
	 
    //Join and exit the playthread
	pthread_join(thread_id, NULL); 
    pthread_exit(NULL);
	
    return 0;
}
/*
 * 	Cleans up the LEDs (writes 0 to them and sets them to inputs)
 */
void cleanup()
{
	printf("Cleaning up\n");

    //TODO
}
/*
 * 	Catch keyboard interrupts and call cleanup before exiting
 */
void ctrlc(int signal)
{
	printf("Program stopping, exiting gracefully\n");
	cleanup();
	exit(0);
}
/*
 * 	Catch aborts and call cleanup before exiting
 */
void catch_abort(int signal)
{
	printf("Caught abort, exiting gracefully\n");
	cleanup();
	exit(1);
}