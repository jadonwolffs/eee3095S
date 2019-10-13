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
#include "project_a.h"

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

void setup()
{
    Blynk.begin(auth, serv, port);
    tmr.setInterval(1000, [](){
      Blynk.virtualWrite(V0, BlynkMillis()/1000);
      FILE *fp;
      char temp_char[255];
      fp = fopen("temp.txt", "r");
      fscanf(fp, "%s", temp_char);
      fclose(fp);
      Blynk.virtualWrite(V1, temp);

      char alarm_char[255];
      fp = fopen("alarm.txt", "r");
      fscanf(fp, "%s", alarm_char);
      fclose(fp);
      // printf("%s\n",alarm_char);
      int alarm_led_val = atoi(alarm_char);
      Blynk.virtualWrite(V2, alarm_led_val);

      char humidity_char[255];
      fp = fopen("humidity.txt", "r");
      fscanf(fp, "%s", humidity_char);
      fclose(fp);
      float humidity_value = atoi(humidity_char);
      Blynk.virtualWrite(V3, humidity_value);

      char light_char[255];
      fp = fopen("light.txt", "r");
      fscanf(fp, "%s", light_char);
      fclose(fp);
      int light_value = atoi(light_char);
      Blynk.virtualWrite(V4, light_value);
      
    });
}

void loop()
{
    Blynk.run();
    tmr.run();
}


int main(int argc, char* argv[])
{
    parse_options(argc, argv, auth, serv, port);

    setup();
    while(true) {
        loop();
    }

    return 0;
}

