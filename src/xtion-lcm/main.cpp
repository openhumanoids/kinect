#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <sys/time.h>
#include <unistd.h>
#include <inttypes.h>
#include <pthread.h>
#include <zlib.h>
#include <glib.h>
#include <lcm/lcm.h>
#include <lcmtypes/kinect_depth_msg_t.h>
#include <lcmtypes/kinect_image_msg_t.h>
#include <lcmtypes/kinect_frame_msg_t.h>
#include <lcmtypes/kinect_cmd_msg_t.h>
#include <lcmtypes/kinect_sensor_status_t.h>
#include <boost/program_options.hpp>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include "jpeg-utils-ijg.h"
#include "timestamp.h"
#include "pixels.h"

namespace po = boost::program_options;

#define alpha (0.05) //decay rate for the moving average
typedef struct _rate_t {
  double target_hz;
  double current_hz;
  int64_t last_tick;
  int64_t tick_count;
} rate_t;

class state_t {
public:
  GThread* freenect_thread;
  volatile int die;
  
  std::string msg_channel;
  std::string status_channel;
  std::string device_id;

  timestamp_sync_state_t* clocksync;
  
  rate_t* capture_rate;
  rate_t* report_rate;
  
  lcm_t* lcm;
  
  pthread_t  work_thread;
    
  int jpeg_quality;
};

rate_t* rate_new(double target_hz)
{
    rate_t* rt = (rate_t *) calloc(1, sizeof(rate_t));
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

    double p_framerate = alpha * (1.0 * 1e6 / dt) + (1 - alpha) * rate->current_hz;
    if (p_framerate > rate->target_hz) {
        // if the potential framerate is too high, don't publish, and return 0
        return 0;
    }
    else {
        // otherwise, update current_hz with a exponential moving average, and return 1
        rate->current_hz = p_framerate;
        rate->last_tick = c_utime;
        rate->tick_count++;
        return 1;
    }
}

void data_cb ( const boost::shared_ptr<openni_wrapper::Image>& image, 
	       const boost::shared_ptr<openni_wrapper::DepthImage>& depthImage, 
	       float constant, state_t* state)
{
  if ( !rate_check(state->capture_rate) ) return;

  int64_t timestamp = timestamp_now();

  unsigned char* rgbBuffer = new unsigned char[image->getWidth() * image->getHeight() * 3];
  image->fillRGB(image->getWidth(), image->getHeight(), rgbBuffer);

  int compressed_size = 640*480*10;
  unsigned char* compressedBuffer = new unsigned char[compressed_size];

  int compression_status = jpegijg_compress_8u_rgb(rgbBuffer, 640, 480, 640*3,
						   compressedBuffer, &compressed_size, state->jpeg_quality);
    
  if (0 != compression_status) {
    fprintf(stderr, "JPEG compression failed...\n");
  }

  float* depthBuffer = new float[640*480];
  depthImage->fillDepthImage(640, 480, depthBuffer);
  short* depthBuffer_mm = new short[640*480];

  for ( int r = 0; r < 480; r++ ) {
    for ( int c = 0; c < 640; c++ ) {
      float v = depthBuffer[r*640+c];
      depthBuffer_mm[r*640+c] = (short)(v*1000.0);
    }
  }

  int uncompressed_size = 640*480*sizeof(short);
  unsigned char* compressedDepthBuffer = new unsigned char[uncompressed_size];
  unsigned long compressedDepthSize = uncompressed_size;
  compress2(compressedDepthBuffer, &compressedDepthSize, (unsigned char*)depthBuffer_mm, 
	    uncompressed_size, Z_BEST_SPEED);

  kinect_frame_msg_t msg;
  msg.timestamp = timestamp;
  msg.image.timestamp = timestamp;
  msg.image.image_data_nbytes = compressed_size;
  msg.image.image_data_format = KINECT_IMAGE_MSG_T_VIDEO_RGB_JPEG;
  msg.image.image_data = compressedBuffer;
  msg.image.width = 640;
  msg.image.height = 480;

  msg.depth.timestamp = timestamp;
  msg.depth.width = 640;
  msg.depth.height = 480;
  msg.depth.depth_data_format = KINECT_DEPTH_MSG_T_DEPTH_MM;
  
  msg.depth.depth_data = compressedDepthBuffer;
  msg.depth.depth_data_nbytes = (int)compressedDepthSize;
  msg.depth.compression = KINECT_DEPTH_MSG_T_COMPRESSION_ZLIB;
  msg.depth.uncompressed_size = uncompressed_size;

  //msg.depth.depth_data = (unsigned char*)depthBuffer_mm;
  //msg.depth.depth_data_nbytes = 480*640*sizeof(short);
  //msg.depth.compression = KINECT_DEPTH_MSG_T_COMPRESSION_NONE;
  //msg.depth.uncompressed_size = msg.depth.depth_data_nbytes;

  kinect_frame_msg_t_publish(state->lcm,  state->msg_channel.c_str(), &msg);

  delete rgbBuffer;
  delete compressedBuffer;
  delete depthBuffer;
  delete depthBuffer_mm;
  delete compressedDepthBuffer;
}

static void *
freenect_threadfunc(void *user_data)
{
    printf("starting kinect thread...\n");
    state_t* state = (state_t*) user_data;

    pcl::OpenNIGrabber* interface = new pcl::OpenNIGrabber(state->device_id);
    std::cout << "connected to device s/n: " << interface->getDevice()->getSerialNumber()
	      << ", at " << interface->getDevice()->getConnectionString() << std::endl;

    boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&, 
			  const boost::shared_ptr<openni_wrapper::DepthImage>&, 
			  float) > callbackHandle = boost::bind (&data_cb, _1, _2, _3, state);

    interface->registerCallback(callbackHandle);

    interface->start ();

    while (!state->die) {
      boost::this_thread::sleep (boost::posix_time::seconds (1));
    }

    interface->stop ();

    printf("-- done!\n");
    return NULL;
}

//pthread - for publishing sensor status 
static void *status_thread(void *user)
{
    state_t *self = (state_t*) user;

    while(1){
        if(self->report_rate){
            kinect_sensor_status_t msg;
            msg.utime = timestamp_now();
            msg.sensor_name = "kinect"; //maybe use some indexing - to make it unique
            msg.rate = self->capture_rate->current_hz;
            
            msg.type = KINECT_SENSOR_STATUS_T_KINECT; //prob need to identify this more - if there are multiple kinects

            kinect_sensor_status_t_publish(self->lcm, self->status_channel.c_str(), &msg);
        }
        sleep(1);
    }
    
    return 0;
}

int main(int argc, char **argv)
{
  state_t *state = new state_t();

  state->freenect_thread = NULL;
  state->die = 0;

  state->jpeg_quality = 94;

  std::string lcm_url;
  double capture_rate;
  state->msg_channel = "KINECT_FRAME";

  // Declare the supported options.
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message")
    ( "channel,c", 
      po::value<std::string>(&state->msg_channel)->default_value(std::string("KINECT_FRAME")), 
      "output RGB-D LCM publish name")
    ( "status,s", 
      po::value<std::string>(&state->status_channel)->default_value(std::string("SENSOR_STATUS_KINECT")), 
      "output status LCM publish name")
    ( "id", 
      po::value<std::string>(&state->device_id)->default_value(std::string("")), 
      "device id (e.g., '#1' or usb location '2@3'")
    ( "lcmurl", 
      po::value<std::string>(&lcm_url)->default_value(std::string("")), 
      "lcm URL" )
    ( "rate,r", 
      po::value<double>(&capture_rate)->default_value(30.0), 
      "maximum publish rate (Hz)" )
    ;
  
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);
  
  if (vm.count("help")) {
    std::cout << desc << std::endl;
    exit(1);
  }

  state->capture_rate = rate_new(capture_rate);
  state->report_rate = rate_new(0.5);
  
  if ( lcm_url == "" ) 
    state->lcm = lcm_create(NULL);
  else
    state->lcm = lcm_create(lcm_url.c_str());  
  if (!state->lcm) {
    fprintf(stderr, "Unable to initialize LCM\n");
    return 1;
  }
  
  // setup passive time synchronization so we can guess the true image
  // acquisition times
  state->clocksync = timestamp_sync_init(100000000, 0xFFFFFFFFLL, 1.001);
  
  if (!g_thread_supported())
    g_thread_init(NULL);
  
  GError *thread_err = NULL;
  state->freenect_thread = g_thread_create(freenect_threadfunc, state, TRUE, &thread_err);
  if (thread_err) {
    fprintf(stderr, "Error creating thread: %s\n", thread_err->message);
    return 1;
  }
  
  pthread_create(&state->work_thread, NULL, status_thread, state);
  
  while (!state->die) {
    lcm_handle(state->lcm);
  }
  
  timestamp_sync_free(state->clocksync);
  rate_destroy(state->capture_rate);
  delete state;
  
  return 0;
}
