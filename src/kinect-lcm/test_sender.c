// file: test_sender.c
//
// Test parameters

#include <stdio.h>
#include <lcm/lcm.h>
#include <unistd.h>
#include "timestamp.h"

#include <lcmtypes/kinect_cmd_t.h>


static void
send_message(lcm_t * lcm, int tilt_angle, int led_state )
{


  int64_t host_utime = timestamp_now();

  if(led_state > 6 || led_state < 0)
    led_state = KINECT_CMD_T_LED_IGNORE;
  
  kinect_cmd_t k_cmd = {
    .timestamp = host_utime,
    .freenect_angle = tilt_angle,
    .freenect_led_status = led_state,
  };

  kinect_cmd_t_publish(lcm, "KINECT_CMD", &k_cmd);
}

int
main(int argc, char ** argv)
{
  int c;
  int tilt_angle = 0;
  int led_state = KINECT_CMD_T_LED_IGNORE;//100;//kinect_cmd_t.LED_IGNORE;  

  while ((c = getopt (argc, argv, "ht:l:")) >= 0) {
    switch (c) {
    case 't':
      tilt_angle = atoi(optarg);
      fprintf(stdout,"Angle : %d\n", tilt_angle);
      
      break;
    case 'l':
      led_state = atoi(optarg);
      fprintf(stdout,"LED state : %d\n", led_state);
      break;
    case 'h':
    case '?':
      fprintf (stderr, "Usage: %s [-v] [-c] [-d] [-p]\n\
                        Options:\n\
                        -t     Tilt Angle\n	\
                        -l     LED state\n\
                        -h     This help message\n", argv[0]);
    return 1;
    }
  }
  
  lcm_t * lcm;

  lcm = lcm_create(NULL);
  if(!lcm)
    return 1;

  send_message(lcm, tilt_angle, led_state);

  lcm_destroy(lcm);
  return 0;
}
