#include "kinectOpenniLCM.h"
#include <boost/math/constants/constants.hpp>
#include <boost/program_options.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>

#include "openni_camera/openni_exception.h"
#include "openni_camera/openni_depth_image.h"

#include <lcmtypes/kinect_depth_msg_t.h>
#include <lcmtypes/kinect_image_msg_t.h>
#include <lcmtypes/kinect_frame_msg_t.h>
#include <lcmtypes/kinect_cmd_msg_t.h>
#include <bot_core/timestamp.h>

#include <zlib.h>
#include <glib.h>

namespace po = boost::program_options;

KinectOpenniLCM::KinectOpenniLCM(int argc, char **argv)
{

  std::string deviceId;

  // Declare the supported options.
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message")
    ( "id", 
      po::value<std::string>(&deviceId)->default_value(std::string("#1")), 
      "index (e.g., #1) or serial of kinect" )
    ;
  
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);
  
  if (vm.count("help")) {
    std::cout << desc << std::endl;
    exit(1);
  }

  m_lcm = bot_lcm_get_global(NULL);

  SetupDevice(deviceId);
}

KinectOpenniLCM::~KinectOpenniLCM()
{
  if ( m_device ) {
    m_device->stopDepthStream ();
    m_device->stopImageStream ();
  }
}

void KinectOpenniLCM::SetupDevice(const std::string& deviceId)
{
  m_device = boost::shared_ptr<openni_wrapper::OpenNIDevice > ((openni_wrapper::OpenNIDevice*)NULL);

  // Initialize the openni device
  openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance ();

  do {

    driver.updateDeviceList ();

    if (driver.getNumberDevices () == 0) {
      std::cout << "No devices connected.... waiting for devices to be connected" << std::endl;
      sleep(1);
      continue;
    }

    std::cout << boost::format("Number devices connected: %d") % driver.getNumberDevices() << std::endl;
    for (unsigned deviceIdx = 0; deviceIdx < driver.getNumberDevices (); ++deviceIdx) {
      std::cout << boost::format("  %u. device on bus %03i:%02i is a %s (%03X) from %s (%03X) with serial id \'%s\'") 
	% (deviceIdx+1) % (int)driver.getBus(deviceIdx) % (int)driver.getAddress(deviceIdx)
	% std::string(driver.getProductName(deviceIdx)) % driver.getProductID(deviceIdx) % std::string(driver.getVendorName (deviceIdx))
	% driver.getVendorID(deviceIdx) % std::string(driver.getSerialNumber(deviceIdx)) << std::endl;
    }

    try {
      if (deviceId[0] == '#') {
	unsigned int index = boost::lexical_cast<unsigned int>(deviceId.substr(1));
	std::cout << boost::format("searching for device with index = %d") % index << std::endl;
	m_device = driver.getDeviceByIndex(index - 1);
	break;
      } else {
	std::cout << boost::format("searching for device with serial number = %s") % deviceId << std::endl;
	m_device = driver.getDeviceBySerialNumber(deviceId);
      }
    }
    catch (const openni_wrapper::OpenNIException& exception) {
      if (!m_device) {
	std::cout << boost::format("No matching device found.... waiting for devices. Reason: %s") % exception.what () << std::endl;
        sleep (1);
        continue;
      } else {
	std::cout << boost::format("could not retrieve device. Reason %s") % exception.what () << std::endl;
        exit(-1);
      } 
    }
  } while ( !m_device );

  std::cout << boost::format("Opened '%s' on bus %i:%i with serial number '%s'") % m_device->getProductName() 
    % (int)m_device->getBus () % (int)m_device->getAddress() % m_device->getSerialNumber() << std::endl;

  m_device->registerImageCallback(&KinectOpenniLCM::ImageCallback, *this);
  m_device->registerDepthCallback(&KinectOpenniLCM::DepthCallback, *this);

  m_device->startImageStream ();
  m_device->startDepthStream ();
  startSynchronization ();
}

void KinectOpenniLCM::startSynchronization ()
{
  if (m_device->isSynchronizationSupported () && !m_device->isSynchronized () &&
      m_device->getImageOutputMode ().nFPS == m_device->getDepthOutputMode ().nFPS &&
      m_device->isImageStreamRunning () && m_device->isDepthStreamRunning () )
    m_device->setSynchronization (true);
}

void KinectOpenniLCM::stopSynchronization ()
{
  if (m_device->isSynchronizationSupported () && m_device->isSynchronized ())
    m_device->setSynchronization (false);
}

void KinectOpenniLCM::ImageCallback (boost::shared_ptr<openni_wrapper::Image> image, void* cookie)
{
  int64_t thisTime = bot_timestamp_now();
  int64_t diffTime = thisTime - m_lastImageTime;
  int64_t diffFromDepthTime = thisTime - m_lastDepthTime;

  //std::cout << "got an image     :" << thisTime << ", " << ((float)diffTime/1000000.0f) << ", " << ((float)diffFromDepthTime/1000000.0f) << std::endl;

  m_lastImageTime = thisTime;
}

void KinectOpenniLCM::DepthCallback (boost::shared_ptr<openni_wrapper::DepthImage> depth_image, void* cookie)
{
  int64_t thisTime = bot_timestamp_now();
  int64_t diffTime = thisTime - m_lastDepthTime;
  int64_t diffFromImageTime = thisTime - m_lastImageTime;

  //std::cout << "got a depth image:" << thisTime << ", " << ((float)diffTime/1000000.0f) << ", " << ((float)diffFromImageTime/1000000.0f) << std::endl;

  m_lastDepthTime = thisTime;

  kinect_frame_msg_t msg;
  msg.timestamp = thisTime;

  msg.image.timestamp = thisTime;
  msg.image.width = 0;
  msg.image.height = 0;
  msg.image.image_data_nbytes = 0;
  msg.image.image_data_format = KINECT_IMAGE_MSG_T_VIDEO_NONE;

  msg.depth.timestamp = thisTime;
  msg.depth.width = depth_image->getWidth();
  msg.depth.height = depth_image->getHeight();
  msg.depth.compression = KINECT_DEPTH_MSG_T_COMPRESSION_NONE;
  msg.depth.depth_data_format = KINECT_DEPTH_MSG_T_DEPTH_11BIT;
  msg.depth.depth_data_nbytes = depth_image->getHeight() * depth_image->getWidth() * sizeof(short);
  msg.depth.uncompressed_size = msg.depth.depth_data_nbytes;
  msg.depth.depth_data = new uint8_t[msg.depth.depth_data_nbytes];
  
  
  depth_image->fillDisparityImage(msg.depth.width, msg.depth.height, reinterpret_cast<unsigned short*>(msg.depth.depth_data), depth_image->getWidth() * sizeof(short));
  /*
  unsigned short* s = (unsigned short*)msg.depth.depth_data;
  unsigned int si = msg.depth.depth_data_nbytes / sizeof(short);
  for ( unsigned int i = 0; i < si; i++ ) {
    std::cout << boost::format("%04X") % s[i] << ", ";
  }
  std::cout << std::endl;
  exit(-1);
  */
  /*
  void* src = (void*)depth_image->getDepthMetaData().Data();
  unsigned long srcSize = depth_image->getDepthMetaData().DataSize();
  void* dest = msg.depth.depth_data;
  unsigned long destSize = msg.depth.depth_data_nbytes;
  compress2((Bytef*)dest, &destSize, (const Bytef*)src, srcSize, Z_BEST_SPEED);
  msg.depth.depth_data_nbytes = (int)destSize;
  msg.depth.compression = KINECT_DEPTH_MSG_T_COMPRESSION_ZLIB;
  */

  /*
  std::cout << "   w=" << msg.depth.width 
	    << ", h="  << msg.depth.height 
	    << ", s=" << msg.depth.depth_data_nbytes << std::endl;
  */
  kinect_frame_msg_t_publish(m_lcm, "KINECT_FRAME", &msg);

  delete msg.depth.depth_data;
}
