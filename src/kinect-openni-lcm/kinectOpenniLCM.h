#ifndef __lidartilt_h__
#define __lidartilt_h__

#include <glib.h>
#include <lcmtypes/bot_core.h>
#include <bot_core/bot_core.h>
//#include <lcmtypes/microstrain_ins_t.h>
#include <iostream>
#include <iomanip>
#include <vector>
#include <list>
#include <string>
#include <boost/circular_buffer.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/shared_ptr.hpp>
#include "openni_camera/openni_device.h"
#include "openni_camera/openni_driver.h"

//------------------------------------
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

#if USE_JPEG_UTILS_POD
#include <jpeg-utils/jpeg-utils.h>
#else
#include "jpeg-utils-ijg.h"
#endif

#include <lcmtypes/kinect_depth_msg_t.h>
#include <lcmtypes/kinect_image_msg_t.h>
#include <lcmtypes/kinect_frame_msg_t.h>

#include "timestamp.h"
#include "pixels.h"

#define dbg(...) fprintf(stderr, __VA_ARGS__)

#define alpha 0.05 //decay rate for the moving average
//----------------------------------


class KinectOpenniLCM
{
 public:

  KinectOpenniLCM(int argc, char **argv);
  virtual ~KinectOpenniLCM();

 private:
  void SetupDevice(const std::string& deviceId);
  void ImageCallback (boost::shared_ptr<openni_wrapper::Image> image, void* cookie);
  void DepthCallback (boost::shared_ptr<openni_wrapper::DepthImage> depth_image, void* cookie);
  void startSynchronization ();
  void stopSynchronization ();
  void usage(const char*);

 private:

  lcm_t* m_lcm;
  int64_t m_lastImageTime;
  int64_t m_lastDepthTime;

  uint8_t* rgb_data;
  bool new_data;

  typedef struct rate_t {
  double target_hz;
  double current_hz;
  int64_t last_tick;
  int64_t tick_count;
  } rate_t;

  void rate_destroy(rate_t* rate);

  /**
   * returns: 1 if an image should be published.  0 if not
   */
  int rate_check(rate_t* rate);

  typedef struct _state_t {
  GThread* openni_thread;
  volatile int die;

  int8_t requested_image_format;
  int8_t current_image_format;

  int8_t requested_depth_format;
  int8_t current_depth_format;

  int8_t current_resolution;

  uint8_t* image_data;

  uint8_t* image_buf;
  int image_buf_size;

  uint8_t* debayer_buf;
  int debayer_buf_size;

  uint16_t* depth_unpack_buf;
  int depth_unpack_buf_size;

  uint8_t* depth_compress_buf;
  int debayer_buf_stride;
  int depth_compress_buf_size;

  double accel_mks[3];
  double tilt_radians;


  kinect_frame_msg_t msg;
  char* msg_channel;

  int got_img;
  int got_depth;

  int skip_img;
  int skip_depth;
  int throttle; //1 in data skip will be published
  timestamp_sync_state_t* clocksync;

  rate_t* capture_rate;
  rate_t* report_rate;

  uint8_t* jpeg_buf;
  int jpeg_buf_size;

  int jpeg_quality;

  int use_zlib;

  lcm_t* lcm;

  int64_t last_timestamp;
  int64_t last_depth_timestamp;
  int64_t last_img_timestamp;
} state_t;

  boost::shared_ptr<openni_wrapper::OpenNIDevice> m_device;

  state_t *state;
};

#endif
