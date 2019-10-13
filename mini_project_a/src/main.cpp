/**
 * @file       main.cpp
 * @author     Volodymyr Shymanskyy
 * @license    This project is released under the MIT License (MIT)
 * @copyright  Copyright (c) 2015 Volodymyr Shymanskyy
 * @date       Mar 2015
 * @brief
 */

//#define BLYNK_DEBUG
#define BLYNK_PRINT stdout
#ifdef RASPBERRY
  #include <BlynkApiWiringPi.h>
#else
  #include <BlynkApiLinux.h>
#endif
#include <BlynkSocket.h>
#include <BlynkOptionsParser.h>

// #include "project_a.h"
// #include "RTC.h"

static BlynkTransportSocket _blynkTransport;
BlynkSocket Blynk(_blynkTransport);

static const char *auth, *serv;
static uint16_t port;

#include <BlynkWidgets.h>

BlynkTimer tmr;

BLYNK_WRITE(V1)
{
    printf("Got a value: %s\n", param[0].asStr());
}

// void * run_env_logger(void *threadargs){
//   run_env();
// }

void setup()
{
    Blynk.begin(auth, serv, port);
    tmr.setInterval(1000, [](){
      Blynk.virtualWrite(V0, BlynkMillis()/1000);
      Blynk.virtualWrite(V1, *temp_ptr);
      // printf("wrote %d to v1\n",*temp_ptr);
    });
}

void loop()
{
    Blynk.run();
    tmr.run();
}


int main(int argc, char* argv[])
{
    // pthread_attr_t tattr;
    // pthread_t thread_id;
    // int newprio = 99;
    // sched_param param;
    // pthread_attr_init (&tattr);
    // pthread_attr_getschedparam (&tattr, &param);
    // param.sched_priority = newprio;
    // pthread_attr_setschedparam (&tattr, &param);
    // pthread_create(&thread_id, &tattr, run_env_logger, (void *)1);
    // parse_options(argc, argv, auth, serv, port);

    setup();
    while(true) {
        loop();
    }

    return 0;
}

