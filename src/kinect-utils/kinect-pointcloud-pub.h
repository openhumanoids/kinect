#ifndef __lidartilt_h__
#define __lidartilt_h__

#include <glib.h>
#include <lcmtypes/bot_core.h>
#include <bot_core/bot_core.h>
#include <iostream>
#include <iomanip>
#include <vector>
#include <list>
#include <string>
#include <boost/circular_buffer.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <lcmtypes/kinect_frame_msg_t.h>
#include "kinect-utils.h"

class KinectPointCloudPub
{
 public:

  KinectPointCloudPub(int argc, char **argv);
  virtual ~KinectPointCloudPub();

  void OnKinectFrame(const lcm_recv_buf_t *rbuf, const char *channel, 
		     const kinect_frame_msg_t *msg);

 private:
  std::string m_kinectFrameName;

  lcm_t* m_lcm;
  kinect_frame_msg_t_subscription_t* m_kinectSubscription;

  std::vector<uint8_t> m_uncompressBuffer;
  std::vector<uint16_t> m_disparity;
  KinectCalibration *m_calib;
};

#endif
