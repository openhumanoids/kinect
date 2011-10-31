#include "kinectOpenniLCM.h"
#include <boost/math/constants/constants.hpp>
#include <boost/program_options.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include "openni_camera/openni_exception.h"

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
  std::cout << "got an image" << std::endl;
}

void KinectOpenniLCM::DepthCallback (boost::shared_ptr<openni_wrapper::DepthImage> depth_image, void* cookie)
{
  std::cout << "got a depth image" << std::endl;
}
