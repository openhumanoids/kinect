#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <sys/time.h>
#include <unistd.h>
#include <inttypes.h>

#include <zlib.h>
#include <glib.h>
#include <lcm/lcm.h>
#include <libfreenect.h>

#include <lcmtypes/kinect_depth_data_t.h>
#include <lcmtypes/kinect_image_data_t.h>
#include <lcmtypes/kinect_cmd_t.h>

#include "jpeg-utils-ijg.h"

#include "timestamp.h"

#define dbg(...) fprintf(stderr, __VA_ARGS__)

#define alpha 0.05 //decay rate for the moving average

typedef struct _rate_t {
  double target_hz;
  double current_hz;
  int64_t last_tick;
  int64_t tick_count;
} rate_t; 

rate_t* rate_new(double target_hz);

void rate_destroy(rate_t* rate);

/**
 * returns: 1 if an image should be published.  0 if not
 */
int rate_check(rate_t* rate); 

typedef struct _state_t {
  GThread* freenect_thread;
  volatile int die;

  freenect_context *f_ctx;
  freenect_device *f_dev;
  int freenect_angle;
  int freenect_led;

  freenect_video_format requested_image_format;
  freenect_video_format current_image_format;

  freenect_depth_format requested_depth_format;
  freenect_depth_format current_depth_format;

  uint8_t* image_data;

  uint8_t* image_buf;
  int image_buf_size;

  uint8_t* depth_buf;
  int depth_buf_size;

  freenect_raw_tilt_state* tilt_state;
  double accel_mks[3];
  double tilt_radians;

  int8_t current_led;
  int8_t requested_led;

  char* image_channel;
  char* depth_channel;

  kinect_image_data_t image_msg;
  kinect_depth_data_t depth_msg;

  int have_img;
  int have_depth;
  int skip_img;
  int throttle; //1 in data skip will be published
  timestamp_sync_state_t* clocksync;

  rate_t* depth_rate;
  rate_t* img_rate;
  rate_t* report_rate;

  uint8_t* jpeg_buf;
  int jpeg_buf_size;

  int jpeg_quality;
  int use_jpeg;

  int use_zlib;

  lcm_t* lcm;
} state_t;

static void
populate_status_t(state_t* state, kinect_status_t* msg, int64_t timestamp)
{
  freenect_update_tilt_state(state->f_dev);
  state->tilt_state = freenect_get_tilt_state(state->f_dev);

  double dx,dy,dz;
  freenect_get_mks_accel(state->tilt_state, &dx, &dy, &dz);

  msg->timestamp = timestamp;
  msg->raw_accel[0] = state->tilt_state->accelerometer_x;
  msg->raw_accel[1] = state->tilt_state->accelerometer_y;
  msg->raw_accel[2] = state->tilt_state->accelerometer_z;
  msg->raw_tilt = state->tilt_state->tilt_angle;

  msg->accel[0] = dx;
  msg->accel[1] = dy;
  msg->accel[2] = dz;
  msg->tilt_radians = freenect_get_tilt_degs(state->tilt_state) * M_PI / 180;

  msg->led_status = state->current_led;

  switch(state->tilt_state->tilt_status) {
    case TILT_STATUS_STOPPED:
      msg->tilt_status = KINECT_STATUS_T_TILT_STATUS_STOPPED;
      break;
    case TILT_STATUS_LIMIT:
      msg->tilt_status = KINECT_STATUS_T_TILT_STATUS_LIMIT;
      break;
    case TILT_STATUS_MOVING:
      msg->tilt_status = KINECT_STATUS_T_TILT_STATUS_MOVING;
      break;
  }
  msg->tilt_status = state->tilt_state->tilt_status;
}

rate_t* rate_new(double target_hz)
{
  rate_t* rt = (rate_t *) calloc(1,sizeof(rate_t));
  rt->target_hz = target_hz;
  rt->tick_count = 0;
  return rt;
}

void rate_destroy(rate_t* rate)
{
  free(rate);
}

int rate_check(rate_t* rate) 
{
  // check the current time
  int64_t c_utime = timestamp_now();

  // compute the framerate if we were to publish an image
  int64_t dt = c_utime - rate->last_tick;

  double p_framerate = alpha * (1.0 * 1e6 / dt)  + (1 - alpha) * rate->current_hz;
  if(p_framerate > rate->target_hz){
    // if the potential framerate is too high, don't publish, and return 0
    return 0;
  } else {
    // otherwise, update current_hz with a exponential moving average, and return 1
    rate->current_hz = p_framerate;
    rate->last_tick = c_utime;
    rate->tick_count++;
    return 1;
  }
}

void cmd_cb(const lcm_recv_buf_t *rbuf __attribute__((unused)), 
      const char *channel __attribute__((unused)), 
      const kinect_cmd_t *msg,
      void *user)
{
  state_t *self = (state_t *)user;

  if(msg->command_type == KINECT_CMD_T_SET_TILT) {
    dbg("Received tilt command; Angle : %d\n", msg->tilt_degree);
    self->freenect_angle = msg->tilt_degree;

    if(self->freenect_angle > 30)
      self->freenect_angle = 30;
    else if(self->freenect_angle < -20)
      self->freenect_angle = -20;

    // XXX is libfreenect thread-safe?
    freenect_set_tilt_degs(self->f_dev,self->freenect_angle);
  } else if(msg->command_type == KINECT_CMD_T_SET_LED) {
    // check that requested LED status is valid
    if(msg->led_status >=0 && msg->led_status <=6) {
      // XXX is libfreenect thread-safe?
      freenect_set_led(self->f_dev, msg->led_status);
    }
  } else if(msg->command_type == KINECT_CMD_T_SET_DEPTH_DATA_FORMAT) {
    // check that requested depth format is valid
    if(msg->depth_data_format >=0 && msg->depth_data_format < 4) {
      self->requested_depth_format = msg->depth_data_format;
    } 
  } else if(msg->command_type == KINECT_CMD_T_SET_IMAGE_DATA_FORMAT) {
    switch(msg->image_data_format) {
      case KINECT_IMAGE_DATA_T_VIDEO_RGB:
        self->requested_image_format = FREENECT_VIDEO_RGB;
        self->use_jpeg = 0;
        break;
      case KINECT_IMAGE_DATA_T_VIDEO_BAYER:
        self->requested_image_format = FREENECT_VIDEO_BAYER;
        break;
      case KINECT_IMAGE_DATA_T_VIDEO_IR_8BIT:
        self->requested_image_format = FREENECT_VIDEO_IR_8BIT;
        break;
      case KINECT_IMAGE_DATA_T_VIDEO_IR_10BIT:
        self->requested_image_format = FREENECT_VIDEO_IR_10BIT;
        break;
      case KINECT_IMAGE_DATA_T_VIDEO_IR_10BIT_PACKED:
        self->requested_image_format = FREENECT_VIDEO_IR_10BIT_PACKED;
        break;
      case KINECT_IMAGE_DATA_T_VIDEO_YUV_RGB:
        self->requested_image_format = FREENECT_VIDEO_YUV_RGB;
        break;
      case KINECT_IMAGE_DATA_T_VIDEO_YUV_RAW:
        self->requested_image_format = FREENECT_VIDEO_YUV_RAW;
        break;
      case KINECT_IMAGE_DATA_T_VIDEO_RGB_JPEG:
        self->requested_image_format = FREENECT_VIDEO_RGB;
        self->use_jpeg = 1;
        break;
      default:
        fprintf(stderr, "Invalid image format requested: %d\n", 
            msg->image_data_format);
        break;
    }
  }
}

void depth_cb(freenect_device *dev, void *data, uint32_t timestamp)
{
  state_t* state = (state_t*) freenect_get_user(dev);
  if(state->depth_rate && !rate_check(state->depth_rate)) {
    return;
  }
  int64_t host_utime = timestamp_now();
  state->depth_msg.timestamp = timestamp_sync(state->clocksync, timestamp, host_utime);

  switch(state->current_depth_format) {
    case FREENECT_DEPTH_11BIT:
      state->depth_msg.depth_data_nbytes = FREENECT_DEPTH_11BIT_SIZE;
      state->depth_msg.depth_data_format = KINECT_DEPTH_DATA_T_DEPTH_11BIT;
      break;
    case FREENECT_DEPTH_10BIT:
      state->depth_msg.depth_data_nbytes = FREENECT_DEPTH_10BIT_SIZE;
      state->depth_msg.depth_data_format = KINECT_DEPTH_DATA_T_DEPTH_10BIT;
      break;
    case FREENECT_DEPTH_11BIT_PACKED:
      state->depth_msg.depth_data_nbytes = FREENECT_DEPTH_11BIT_PACKED_SIZE;
      state->depth_msg.depth_data_format = KINECT_DEPTH_DATA_T_DEPTH_11BIT_PACKED;
      break;
    case FREENECT_DEPTH_10BIT_PACKED:
      state->depth_msg.depth_data_nbytes = FREENECT_DEPTH_10BIT_PACKED_SIZE;
      state->depth_msg.depth_data_format = KINECT_DEPTH_DATA_T_DEPTH_10BIT_PACKED;
      break;
    default:
      state->depth_msg.depth_data_nbytes = 0;
      state->depth_msg.depth_data_format = 0; // TODO spew warning

      break;
  }

  if(state->use_zlib) {
      int uncompressed_size = state->depth_msg.depth_data_nbytes;
      unsigned long compressed_size = state->depth_buf_size;
      compress2(state->depth_buf, &compressed_size, data, uncompressed_size, Z_BEST_SPEED);
      state->depth_msg.depth_data_nbytes = (int)compressed_size;
      state->depth_msg.compression = KINECT_DEPTH_DATA_T_COMPRESSION_ZLIB;
      state->depth_msg.uncompressed_size = uncompressed_size;
  } else {
      assert(state->depth_msg.depth_data_nbytes < state->depth_buf_size);
      memcpy(state->depth_buf, data, state->depth_msg.depth_data_nbytes);
      state->depth_msg.compression = KINECT_DEPTH_DATA_T_COMPRESSION_NONE;
      state->depth_msg.uncompressed_size = state->depth_msg.depth_data_nbytes;
  }

  state->have_depth++;
}

void image_cb(freenect_device *dev, void *data, uint32_t timestamp)
{
  state_t* state = (state_t*) freenect_get_user(dev);

  if(state->skip_img)
    return;

  if(state->img_rate && !rate_check(state->img_rate)){
    return;
  }

  int64_t host_utime = timestamp_now();
  state->image_msg.timestamp = timestamp_sync(state->clocksync, timestamp, host_utime);

  switch(state->current_image_format) {
    case FREENECT_VIDEO_RGB:
      state->image_msg.image_data_nbytes = FREENECT_VIDEO_RGB_SIZE;
      state->image_msg.image_data_format = KINECT_IMAGE_DATA_T_VIDEO_RGB;
      break;
    case FREENECT_VIDEO_BAYER:
      state->image_msg.image_data_nbytes =  FREENECT_VIDEO_BAYER_SIZE;
      state->image_msg.image_data_format = KINECT_IMAGE_DATA_T_VIDEO_BAYER;
      break;
    case FREENECT_VIDEO_YUV_RGB:
      state->image_msg.image_data_nbytes =  FREENECT_VIDEO_YUV_RGB_SIZE;
      state->image_msg.image_data_format = KINECT_IMAGE_DATA_T_VIDEO_YUV_RGB;
      break;
    case FREENECT_VIDEO_YUV_RAW:
      state->image_msg.image_data_nbytes =  FREENECT_VIDEO_YUV_RAW_SIZE;
      state->image_msg.image_data_format = KINECT_IMAGE_DATA_T_VIDEO_YUV_RAW;
      break;
    case FREENECT_VIDEO_IR_8BIT:
      state->image_msg.image_data_nbytes =  FREENECT_VIDEO_IR_8BIT_SIZE;
      state->image_msg.image_data_format = KINECT_IMAGE_DATA_T_VIDEO_IR_8BIT;
      break;
    case FREENECT_VIDEO_IR_10BIT:
      state->image_msg.image_data_nbytes =  FREENECT_VIDEO_IR_10BIT_SIZE;
      state->image_msg.image_data_format = KINECT_IMAGE_DATA_T_VIDEO_IR_10BIT;
      break;
    case FREENECT_VIDEO_IR_10BIT_PACKED:
      state->image_msg.image_data_nbytes =  FREENECT_VIDEO_IR_10BIT_PACKED_SIZE;
      state->image_msg.image_data_format = KINECT_IMAGE_DATA_T_VIDEO_IR_10BIT_PACKED;
      break;
    default:
      state->image_msg.image_data_nbytes = 0; // TODO spew warning
      state->image_msg.image_data_format = -1;
      break;
  }

  if(state->use_jpeg && state->current_image_format == FREENECT_VIDEO_RGB) {
    int compressed_size = state->image_buf_size;
#if 0
    int compression_status = jpeg_compress_8u_rgb (data, 640, 480, 640*3,
        state->image_buf, &compressed_size, state->jpeg_quality);
#else
    int compression_status = jpegijg_compress_8u_rgb (data, 640, 480, 640*3,
        state->image_buf, &compressed_size, state->jpeg_quality);
#endif

    if(0 != compression_status) {
      fprintf(stderr, "JPEG compression failed...\n");
    }
    state->image_msg.image_data_nbytes = compressed_size;
    state->image_msg.image_data_format = KINECT_IMAGE_DATA_T_VIDEO_RGB_JPEG;

  } else {
    assert(state->image_msg.image_data_nbytes < state->image_buf_size);
    memcpy(state->image_buf, data, state->image_msg.image_data_nbytes);
  }
  state->have_img++;
}

static void *
freenect_threadfunc(void *user_data)
{
  printf("starting kinect thread...\n");
  state_t* state = (state_t*) user_data;

  freenect_set_tilt_degs(state->f_dev, state->freenect_angle);
  freenect_set_led(state->f_dev, state->current_led);
  freenect_set_depth_callback(state->f_dev, depth_cb);
  freenect_set_video_callback(state->f_dev, image_cb);
  freenect_set_video_format(state->f_dev, state->current_image_format);
  freenect_set_depth_format(state->f_dev, state->current_depth_format);
  freenect_set_video_buffer(state->f_dev, state->image_data);

  freenect_start_depth(state->f_dev);
  freenect_start_video(state->f_dev);

  state->have_img = 0;
  state->have_depth = 0;

  while (!state->die && freenect_process_events(state->f_ctx) >= 0) {
    if(state->have_img && !state->skip_img){ 
      populate_status_t(state, &state->image_msg.status, state->image_msg.timestamp);
      kinect_image_data_t_publish(state->lcm, state->image_channel, &state->image_msg);
      state->have_img = 0;
    } else if(state->have_depth){
      populate_status_t(state, &state->depth_msg.status, state->depth_msg.timestamp);
      kinect_depth_data_t_publish(state->lcm, state->depth_channel, &state->depth_msg);
      state->have_depth = 0;
    }
    if (state->requested_image_format != state->current_image_format) {
      dbg("Changing Image format\n");
      freenect_stop_video(state->f_dev);
      freenect_set_video_format(state->f_dev, state->requested_image_format);
      freenect_start_video(state->f_dev);
      state->current_image_format = state->requested_image_format;
    }
    if (state->requested_depth_format != state->current_depth_format) {
      dbg("Changing Depth format\n");
      freenect_stop_depth(state->f_dev);
      freenect_set_depth_format(state->f_dev, state->requested_depth_format);
      freenect_start_depth(state->f_dev);
      state->current_depth_format = state->requested_depth_format;
    }

    if(rate_check(state->report_rate)) {
      printf("Image: %6"PRId64" (%.2fHz)  Depth: %6"PRId64" (%.2fHz)\n", 
          state->img_rate->tick_count,
          state->img_rate->current_hz,
          state->depth_rate->tick_count,
          state->depth_rate->current_hz);
    }
  }

  printf("\nshutting down streams...\n");

  freenect_stop_depth(state->f_dev);
  freenect_stop_video(state->f_dev);

  freenect_close_device(state->f_dev);
  freenect_shutdown(state->f_ctx);

  printf("-- done!\n");
  return NULL;
}

static void usage(const char* progname)
{
  fprintf (stderr, "Usage: %s [options]\n"
                   "\n"
                   "Options:\n"
                   "  -r RATE   Throttle publishing to RATE Hz.\n"
//                   "  -d        Do not publish depth images\n"
                   "  -i        Do not publish RGB/infrared images\n"
                   "  -j        JPEG-compress RGB images\n"
                   "  -q QUAL   JPEG compression quality (0-100, default 94)\n"
                   "  -h        This help message\n", 
                   g_path_get_basename(progname));
  exit(1);
}

int main(int argc, char **argv)
{
  state_t *state = (state_t*)calloc(1, sizeof(state_t));

  double target_rate = INFINITY;

  state->skip_img = 0;
  state->throttle = 0;
  state->depth_rate = NULL;
  state->img_rate = NULL;

  state->freenect_thread = NULL;
  state->die = 0;

  state->f_ctx = NULL;
  state->f_dev = NULL;
  state->freenect_angle = 0;
  state->freenect_led = 0;

  state->use_jpeg = 0;
  state->jpeg_quality = 94;

  // make these configurable - either/both from the command line and from outside LCM command
  state->requested_image_format = FREENECT_VIDEO_RGB;
  state->requested_depth_format = FREENECT_DEPTH_11BIT_PACKED;
  state->requested_led = LED_RED;
  state->current_image_format = state->requested_image_format;
  state->current_depth_format = state->requested_depth_format;
  state->current_led = state->requested_led;

  int c;
  // command line options - to throtle - to ignore image publish  
  while ((c = getopt (argc, argv, "hir:jq:z")) >= 0) {
    switch (c) {
      case 'i': //ignore images 
        state->skip_img = 1;
        dbg("Skipping image publishing\n");
        break;
      case 'j':
        state->use_jpeg = 1;
        printf("JPEG compressing RGB data\n");
        break;
      case 'q':
        state->jpeg_quality = atoi(optarg);
        if(state->jpeg_quality < 0 || state->jpeg_quality > 100)
          usage(argv[0]);
        break;
      case 'z':
        state->use_zlib = 1;
        printf("ZLib compressing depth data\n");
        break;
      case 'r':
        target_rate = strtod (optarg,NULL);
        printf("Target Rate is : %f Hz\n", target_rate); 
        state->throttle = 1;
        break;

        //add option to select the image/depth type 

      case 'h':
      case '?':
        usage(argv[0]);
    }
  }

  // 
  int user_device_number = 0;
  if (argc > 1)
    user_device_number = atoi(argv[1]);

  // throttling
  state->depth_rate = rate_new(target_rate);
  state->img_rate = rate_new(target_rate);

  // allocate image and depth buffers
  state->image_data   = (uint8_t*) malloc(640*480*4);

  // allocate more space for the image buffer, as we might use it for compressed data
  state->image_buf_size = 640 * 480 * 10;
  state->image_buf = (uint8_t*) malloc(state->image_buf_size);
  state->image_msg.image_data = state->image_buf;
  state->image_msg.width = FREENECT_FRAME_W;
  state->image_msg.height = FREENECT_FRAME_H;

  // allocate more space for the depth buffer, as we might use it for compressed data
  state->depth_buf_size = 640 * 480 * sizeof(int16_t) * 4;
  state->depth_buf = (uint8_t*) malloc(state->depth_buf_size);
  state->depth_msg.depth_data = state->depth_buf;
  state->depth_msg.width = FREENECT_FRAME_W;
  state->depth_msg.height = FREENECT_FRAME_H;

  state->have_depth = 0;
  state->have_img = 0;

  state->report_rate = rate_new(1.0);

  // initialize LCM
  state->image_channel = g_strdup("KINECT_IMAGE");
  state->depth_channel = g_strdup("KINECT_DEPTH");

  state->lcm = lcm_create(NULL);

  if(!state->lcm) {
    fprintf(stderr, "Unable to initialize LCM\n");
    return 1;
  }

  // initialize the kinect device
  if(freenect_init(&state->f_ctx, NULL) < 0) {
    printf("freenect_init() failed\n");
    return 1;
  }

  freenect_set_log_level(state->f_ctx, FREENECT_LOG_INFO);

  int num_devices = freenect_num_devices(state->f_ctx);
  printf("Number of devices found: %d\n", num_devices);

  if (num_devices < 1)
    return 1;

  if (freenect_open_device(state->f_ctx, &state->f_dev, user_device_number) < 0) {
    printf("Could not open device\n");
    return 1;
  }

  freenect_set_user(state->f_dev, state);

  // setup passive time synchronization so we can guess the true image
  // acquisition times
  state->clocksync = timestamp_sync_init(1000000, 0xFFFFFFFFLL, 1.001);

  g_thread_init(NULL);

  // subscribe to kinect command messages
  kinect_cmd_t_subscribe(state->lcm, "KINECT_CMD", cmd_cb, state);

  GError *thread_err = NULL;
  state->freenect_thread = g_thread_create(freenect_threadfunc, state, TRUE, &thread_err);
  if(thread_err) {
    fprintf(stderr, "Error creating thread: %s\n", thread_err->message);
    return 1;
  }

  while(!state->die) {
    lcm_handle(state->lcm);
  }

  timestamp_sync_free(state->clocksync);
  free(state->image_channel);
  free(state->depth_channel);
  rate_destroy(state->depth_rate);
  rate_destroy(state->img_rate);
  free(state);

  return 0;
}
