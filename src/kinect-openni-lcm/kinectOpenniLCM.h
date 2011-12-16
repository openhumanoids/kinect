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
  uint8_t* depth_data;
  bool new_data;

  boost::shared_ptr<openni_wrapper::OpenNIDevice> m_device;

};

#endif
