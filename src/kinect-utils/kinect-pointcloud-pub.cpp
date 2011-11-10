#include "kinect-pointcloud-pub.h"
#include <boost/math/constants/constants.hpp>
#include <boost/program_options.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <lcmtypes/kinect_frame_msg_t.h>
#include <lcmtypes/kinect_pointcloud_t.h>
#include <algorithm>
#include <zlib.h>

namespace po = boost::program_options;

void onKinectFrame(const lcm_recv_buf_t *rbuf, const char *channel, 
		   const kinect_frame_msg_t *msg, void *user_data)
{
  return ((KinectPointCloudPub*)user_data)->OnKinectFrame(rbuf, channel, msg);
}

KinectPointCloudPub::KinectPointCloudPub(int argc, char **argv) 
{
  // Declare the supported options.
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message")
    ( "c", 
      po::value<std::string>(&m_kinectFrameName)->default_value(std::string("KINECT_FRAME")), 
      "input kinect LCM subscribe name")
    ;
  
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);
  
  if (vm.count("help")) {
    std::cout << desc << std::endl;
    exit(1);
  }

  m_lcm = bot_lcm_get_global(NULL);

  m_kinectSubscription = kinect_frame_msg_t_subscribe(m_lcm, m_kinectFrameName.c_str(), onKinectFrame, this);

  m_calib = kinect_calib_new();
  m_calib->width = 640;
  m_calib->height = 480;

  m_calib->intrinsics_depth.fx = 576.09757860;
  m_calib->intrinsics_depth.cx = 321.06398107;
  m_calib->intrinsics_depth.cy = 242.97676897;
  
  m_calib->intrinsics_rgb.fx = 528.49404721;
  m_calib->intrinsics_rgb.cx = 319.50000000;
  m_calib->intrinsics_rgb.cy = 239.50000000;
  m_calib->intrinsics_rgb.k1 = 0;
  m_calib->intrinsics_rgb.k2 = 0;

  m_calib->shift_offset = 1093.4753;
  m_calib->projector_depth_baseline = 0.07214;;
  
  double R[9] = { 0.999999, -0.000796, 0.001256, 0.000739, 0.998970, 0.045368, -0.001291, -0.045367, 0.998970 };
  double T[3] = { -0.015756, -0.000923, 0.002316 };

  memcpy(m_calib->depth_to_rgb_rot, R, 9*sizeof(double));
  memcpy(m_calib->depth_to_rgb_translation, T, 3*sizeof(double));
}

KinectPointCloudPub::~KinectPointCloudPub()
{
  if ( m_kinectSubscription ) kinect_frame_msg_t_unsubscribe(m_lcm, m_kinectSubscription);
  kinect_calib_destroy(m_calib);
}

void KinectPointCloudPub::OnKinectFrame(const lcm_recv_buf_t *rbuf, const char *channel, 
					const kinect_frame_msg_t *msg)
{
  uint16_t* depth_data = (uint16_t*)msg->depth.depth_data;

  if(msg->depth.compression != KINECT_DEPTH_MSG_T_COMPRESSION_NONE) {

    m_uncompressBuffer.resize(msg->depth.uncompressed_size);

    unsigned long dlen = msg->depth.uncompressed_size;
    int status = uncompress(&m_uncompressBuffer[0], &dlen, msg->depth.depth_data, msg->depth.depth_data_nbytes);
    if(status != Z_OK) {
      return;
    }
    depth_data = (uint16_t*)&m_uncompressBuffer[0];

  }

  if ( msg->depth.depth_data_format != KINECT_DEPTH_MSG_T_DEPTH_11BIT ) {
    std::cout << "ERROR: unable to handle anything but 11 bit depth data" << std::endl;
    return;
  }

  //double depth_to_rgb_uvd[12];
  double depth_to_depth_xyz[16];
  //kinect_calib_get_depth_uvd_to_rgb_uvw_3x4(m_calib, depth_to_rgb_uvd);
  kinect_calib_get_depth_uvd_to_depth_xyz_4x4(m_calib, depth_to_depth_xyz);

  //double depth_to_depth_xyz_trans[16];
  // _matrix_transpose_4x4d(depth_to_depth_xyz, depth_to_depth_xyz_trans);


  std::vector<kinect_point3d_t> points;
  points.reserve(msg->depth.width * msg->depth.height);

  for ( int u = 0; u < msg->depth.width; u++ ) {
    for ( int v = 0; v < msg->depth.height; v++ ) {
      double pix[] = { u, v, depth_data[v*msg->depth.width + u], 1.0 };
      double xyz[4];
      bot_matrix_vector_multiply_4x4_4d (depth_to_depth_xyz, pix, xyz);
      //bot_matrix_print(depth_to_depth_xyz,4,4);
      kinect_point3d_t p;
      p.x = xyz[0]/xyz[3];
      p.y = xyz[1]/xyz[3];
      p.z = xyz[2]/xyz[3];
      points.push_back(p);
      //kinect_point3d_t q(p);
      //if ( u ==0 && v==0 ) {
      //	std::cout << p.x << "; " << p.y << "; " << p.z << std::endl;
      //	std::cout << q.x << ": " << q.y << ": " << q.z << std::endl;
      //}
      //std::cout << xyz[0] << ", " << xyz[1] << ", " << xyz[2] << ", " << xyz[3] << std::endl;
    }
  }

  kinect_pointcloud_t kpc;
  kpc.timestamp = msg->timestamp;
  kpc.points_nbytes = points.size();
  kpc.points = &points[0];

  //std::cout << points[0].x << ", " << points[0].y << ", " << points[0].z << std::endl;

  kinect_pointcloud_t_publish(m_lcm, "POINTS", &kpc);
}


