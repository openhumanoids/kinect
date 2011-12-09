#include "kinectOpenniLCM.h"
#include <boost/math/constants/constants.hpp>
#include <boost/program_options.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>

#include "openni_camera/openni_exception.h"
#include "openni_camera/openni_depth_image.h"
#include "openni_camera/openni_image.h"

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


  state = (state_t*) calloc(1, sizeof(state_t));

  double target_rate = INFINITY;

  state->skip_img = 0;
  state->skip_depth = 0;
  state->throttle = 0;
  state->capture_rate = NULL;

  state->openni_thread = NULL;
  state->die = 0;

  state->jpeg_quality = 94;

  // make these configurable - either/both from the command line and from outside LCM command
  state->requested_image_format = KINECT_IMAGE_MSG_T_VIDEO_BAYER;
  state->requested_depth_format = KINECT_DEPTH_MSG_T_DEPTH_11BIT;
  state->current_image_format = state->requested_image_format;
  state->current_depth_format = state->requested_depth_format;
  int user_device_number = 0;
  state->msg_channel = g_strdup("KINECT_FRAME");

  int c;
  char *lcm_url = NULL;
  // command line options - to throtle - to ignore image publish  
  while ((c = getopt(argc, argv, "hd:i:r:jq:zl:n:c:")) >= 0) {
    switch (c) {
    case 'i': //ignore images
      state->requested_image_format = atoi(optarg);
      break;
    case 'd':
      state->requested_depth_format = atoi(optarg);
      break;
    case 'j':
      state->requested_image_format = KINECT_IMAGE_MSG_T_VIDEO_RGB_JPEG;
      break;
    case 'q':
      state->jpeg_quality = atoi(optarg);
      if (state->jpeg_quality < 0 || state->jpeg_quality > 100)
        usage(argv[0]);
      break;
    case 'n':
      user_device_number = atoi(optarg);
      printf("attempting to open device %i\n", user_device_number);
      break;
    case 'z':
      state->use_zlib = 1;
      printf("ZLib compressing depth data\n");
      break;
    case 'r':
      target_rate = strtod(optarg, NULL);
      printf("Target Rate is : %.3f Hz\n", target_rate);
      state->throttle = 1;
      break;
    case 'l':
      lcm_url = strdup(optarg);
      printf("Using LCM URL \"%s\"\n", lcm_url);
      break;
    case 'c':
      g_free(state->msg_channel);
      state->msg_channel = g_strdup(optarg);
      printf("Output on LCM channel: %s\n", state->msg_channel);
      break;
    case 'h':
      case '?':
      usage(argv[0]);
    }
  }

  const char * depthModeSting[] = {
      "DEPTH_11BIT",
      "DEPTH_10BIT",
      "DEPTH_11BIT_PACKED",
      "DEPTH_10BIT_PACKED",
      "DEPTH_REGISTERED",
      "DEPTH_MM",
  };
  switch (state->requested_depth_format) {
  case KINECT_DEPTH_MSG_T_DEPTH_11BIT:
    case KINECT_DEPTH_MSG_T_DEPTH_10BIT:
    case KINECT_DEPTH_MSG_T_DEPTH_11BIT_PACKED:
    case KINECT_DEPTH_MSG_T_DEPTH_10BIT_PACKED:
    case KINECT_DEPTH_MSG_T_DEPTH_REGISTERED:
    case KINECT_DEPTH_MSG_T_DEPTH_MM:
    dbg("Depth Mode is %s\n", depthModeSting[state->requested_depth_format]);
    break;
  case KINECT_DEPTH_MSG_T_DEPTH_NONE:
    case -1:
    dbg("Depth is disabled");
    state->requested_depth_format = KINECT_DEPTH_MSG_T_DEPTH_NONE;
    state->skip_depth = 1;
    break;
  default:
    dbg("Invalid depth format %d\n", state->requested_depth_format);
    usage(argv[0]);
    break;
  }

  const char * imageModeSting[] = {
      "VIDEO_RGB",
      "VIDEO_BAYER",
      "VIDEO_IR_8BIT",
      "VIDEO_IR_10BIT",
      "VIDEO_IR_10BIT_PACKED",
      "VIDEO_YUV_RGB",
      "VIDEO_YUV_RAW",
  };

  switch (state->requested_image_format) {
  case KINECT_IMAGE_MSG_T_VIDEO_RGB:
    case KINECT_IMAGE_MSG_T_VIDEO_BAYER:
    case KINECT_IMAGE_MSG_T_VIDEO_IR_8BIT:
    case KINECT_IMAGE_MSG_T_VIDEO_IR_10BIT:
    case KINECT_IMAGE_MSG_T_VIDEO_IR_10BIT_PACKED:
    case KINECT_IMAGE_MSG_T_VIDEO_YUV_RGB:
    case KINECT_IMAGE_MSG_T_VIDEO_YUV_RAW:
    dbg("Image Mode is %s\n", imageModeSting[state->requested_image_format]);
    break;

  case KINECT_IMAGE_MSG_T_VIDEO_RGB_JPEG:
    dbg("Jpeg Compressing RGB images\n");
    break;
  case KINECT_IMAGE_MSG_T_VIDEO_NONE:
    case -1:
    dbg("Image is disabled");
    state->requested_image_format = KINECT_IMAGE_MSG_T_VIDEO_NONE;
    state->skip_img = 1;
    break;
  default:
    dbg("Invalid image format %d\n", state->requested_image_format);
    usage(argv[0]);
    break;
  }

  // throttling
  state->capture_rate = (rate_t *) calloc(1, sizeof(rate_t));
  state->capture_rate->target_hz = target_rate;
  state->capture_rate->tick_count = 0;

  //todo the width and height should be computed from the desired resolution

  // allocate image and depth buffers
  state->image_data = (uint8_t*) malloc(640 * 480 * 4);

  // allocate more space for the image buffer, as we might use it for compressed data
  state->image_buf_size = 640 * 480 * 10;
  if (0 != posix_memalign((void**) &state->image_buf, 16, state->image_buf_size)) {
    fprintf(stderr, "Error allocating image buffer\n");
    exit(1);
  }
  state->msg.image.image_data = state->image_buf;
  state->msg.image.width = 640; //TODO: this shouldn't be hardcoded
  state->msg.image.height = 480;

  // allocate a buffer for bayer de-mosaicing
  state->debayer_buf_size = 640 * 480 * 4;
  state->debayer_buf_stride = 640 * 4;
  if (0 != posix_memalign((void**) &state->debayer_buf, 16, state->debayer_buf_size)) {
    fprintf(stderr, "error allocating de-Bayer buffer\n");
    exit(1);
  }

  // allocate space for unpacking depth data
  state->depth_unpack_buf_size = 640 * 480 * sizeof(uint16_t);
  state->depth_unpack_buf = (uint16_t*) malloc(state->depth_unpack_buf_size);

  // allocate space for zlib compressing depth data
  state->depth_compress_buf_size = 640 * 480 * sizeof(int16_t) * 4;
  state->depth_compress_buf = (uint8_t*) malloc(state->depth_compress_buf_size);
  state->msg.depth.depth_data = state->depth_compress_buf;
  state->msg.depth.width = 640;
  state->msg.depth.height = 480;

  state->got_img = 0;
  state->got_depth = 0;

  state->report_rate = (rate_t *) calloc(1, sizeof(rate_t));
  state->report_rate->target_hz = 0.5;
  state->report_rate->tick_count = 0;

  state->last_timestamp = 0;



  SetupDevice(deviceId);

  new_data = false;
}

KinectOpenniLCM::~KinectOpenniLCM()
{
  if ( m_device ) {
    m_device->stopDepthStream ();
    m_device->stopImageStream ();
  }
}

void KinectOpenniLCM::usage(const char* progname)
{
  fprintf(stderr, "Usage: %s [options]\n"
      "\n"
      "Options:\n"
      "  -r RATE   Throttle publishing to RATE Hz.\n"
      "  -d        Depth mode\n"
      "  -i        Image mode\n"
      "  -j        JPEG-compress RGB images\n"
      "  -q QUAL   JPEG compression quality (0-100, default 94)\n"
      "  -z        ZLib compress depth images\n"
      "  -l URL    Specify LCM URL\n"
      "  -h        This help message\n"
      "  -n dev    Number of the device to open\n"
      "  -c name   LCM channel\n",
      g_path_get_basename(progname));

  fprintf(stderr, "Image mode must be one of:\n"
      "  VIDEO_RGB             = 0\n"
      "  VIDEO_BAYER           = 1\n"
      "  VIDEO_IR_8BIT         = 2\n"
      "  VIDEO_IR_10BIT        = 3\n"
      "  VIDEO_IR_10BIT_PACKED = 4\n"
      "  VIDEO_YUV_RGB         = 5\n"
      "  VIDEO_YUV_RAW         = 6\n"
      "\n"
      "  VIDEO_DISABLED        = -1\n"
      );

  fprintf(stderr, "Depth mode must be one of:\n"
      "  DEPTH_11BIT        = 0\n"
      "  DEPTH_10BIT        = 1\n"
      "  DEPTH_11BIT_PACKED = 2\n"
      "  DEPTH_10BIT_PACKED = 3\n"
      "  DEPTH_REGISTERED   = 4\n"
      "  DEPTH_MM           = 5\n"
      "\n"
      "  DEPTH_DISABLED         =-1\n"
      );
  exit(1);
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

  //setting the view point to be the RGB camera 
  m_device->depth_generator_.GetAlternativeViewPointCap().SetViewPoint(m_device->image_generator_);

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

  if (new_data == false)
  {
    rgb_data = (uint8_t*)calloc(640*480*3, sizeof(uint8_t));
    image->fillRGB(image->getWidth(), image->getHeight(), reinterpret_cast<unsigned char*> (rgb_data), 640*3);
    new_data = true;
  }
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
//  new_data = false;
  if (new_data)
  {
    msg.image.timestamp = thisTime;
    msg.image.width = 640;
    msg.image.height = 480;
    msg.image.image_data_nbytes = 640*480*3;
    msg.image.image_data_format = KINECT_IMAGE_MSG_T_VIDEO_RGB;
    msg.image.image_data = rgb_data;
  }
  else
  {
    msg.image.timestamp = thisTime;
    msg.image.width = 0;
    msg.image.height = 0;
    msg.image.image_data_nbytes = 0;
    msg.image.image_data_format = KINECT_IMAGE_MSG_T_VIDEO_NONE;
  }

  msg.depth.timestamp = thisTime;
  msg.depth.width = depth_image->getWidth();
  msg.depth.height = depth_image->getHeight();
  msg.depth.compression = KINECT_DEPTH_MSG_T_COMPRESSION_NONE;
  msg.depth.depth_data_format = KINECT_DEPTH_MSG_T_DEPTH_MM;//KINECT_DEPTH_MSG_T_DEPTH_11BIT;
  msg.depth.depth_data_nbytes = depth_image->getHeight() * depth_image->getWidth() * sizeof(short);
  msg.depth.uncompressed_size = msg.depth.depth_data_nbytes;
  msg.depth.depth_data = (uint8_t*)calloc(msg.depth.depth_data_nbytes, sizeof(uint8_t));
//new uint8_t[msg.depth.depth_data_nbytes];
  //memset(msg.depth.depth_data, 128, msg.depth.depth_data_nbytes/2);    
  //memset(&msg.depth.depth_data[msg.depth.depth_data_nbytes/2], 255, msg.depth.depth_data_nbytes/2);    
  depth_image->fillDepthImageRaw(msg.depth.width, msg.depth.height, reinterpret_cast<unsigned short*>(msg.depth.depth_data), depth_image->getWidth() * sizeof(short));

  kinect_frame_msg_t_publish(m_lcm, "KINECT_FRAME", &msg);
  free(msg.depth.depth_data);
  if (new_data) 
    free(rgb_data);
  new_data = false;
}
