#ifndef __lidartilt_h__
#define __lidartilt_h__

#include <glib.h>
#include <lcmtypes/bot_core.h>
#include <bot_core/bot_core.h>
#include <lcmtypes/microstrain_ins_t.h>
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

 private:

  lcm_t* m_lcm;
  int64_t m_lastImageTime;
  int64_t m_lastDepthTime;

  boost::shared_ptr<openni_wrapper::OpenNIDevice> m_device;
};

#endif
