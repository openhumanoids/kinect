#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <sys/time.h>

#include <glib.h>
#include <lcm/lcm.h>
#include <libfreenect.h>

#include <lcmtypes/kinect_depth_data_t.h>
#include <lcmtypes/kinect_image_data_t.h>

#define dbg(...) fprintf(stderr, __VA_ARGS__)

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

    uint8_t* depth_data;
    uint8_t* image_data;

    freenect_raw_tilt_state* tilt_state;
    double accel_mks[3];
    double tilt_radians;

    int8_t current_led;
    int8_t requested_led;

    char* image_channel;
    char* depth_channel;

    lcm_t* lcm;
} state_t;

static int64_t _timestamp_now()
{
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

#if 0

void keyPressed(unsigned char key, int x, int y)
{
	if (key == 27) {
		die = 1;
		pthread_join(freenect_thread, NULL);
		glutDestroyWindow(window);
		free(depth_data);
		free(depth_front);
		free(image_data);
		free(rgb_mid);
		free(rgb_front);
		pthread_exit(NULL);
	}
	if (key == 'w') {
		freenect_angle++;
		if (freenect_angle > 30) {
			freenect_angle = 30;
		}
	}
	if (key == 's') {
		freenect_angle = 0;
	}
	if (key == 'f') {
		if (requested_image_format == FREENECT_VIDEO_IR_8BIT)
			requested_image_format = FREENECT_VIDEO_RGB;
		else if (requested_image_format == FREENECT_VIDEO_RGB)
			requested_image_format = FREENECT_VIDEO_YUV_RGB;
		else
			requested_image_format = FREENECT_VIDEO_IR_8BIT;
	}
	if (key == 'x') {
		freenect_angle--;
		if (freenect_angle < -30) {
			freenect_angle = -30;
		}
	}
	if (key == '1') {
		freenect_set_led(f_dev,LED_GREEN);
	}
	if (key == '2') {
		freenect_set_led(f_dev,LED_RED);
	}
	if (key == '3') {
		freenect_set_led(f_dev,LED_YELLOW);
	}
	if (key == '4') {
		freenect_set_led(f_dev,LED_BLINK_YELLOW);
	}
	if (key == '5') {
		freenect_set_led(f_dev,LED_BLINK_GREEN);
	}
	if (key == '6') {
		freenect_set_led(f_dev,LED_BLINK_RED_YELLOW);
	}
	if (key == '0') {
		freenect_set_led(f_dev,LED_OFF);
	}
	freenect_set_tilt_degs(f_dev,freenect_angle);
}

uint16_t t_gamma[2048];
#endif

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

void depth_cb(freenect_device *dev, void *data, uint32_t timestamp)
{
    state_t* state = (state_t*) freenect_get_user(dev);

    kinect_depth_data_t msg;
    msg.timestamp = _timestamp_now();

    switch(state->current_depth_format) {
        case FREENECT_DEPTH_11BIT:
            msg.depth_data_nbytes = FREENECT_DEPTH_11BIT_SIZE;
            msg.depth_data_format = KINECT_DEPTH_DATA_T_DEPTH_11BIT;
            break;
        case FREENECT_DEPTH_10BIT:
            msg.depth_data_nbytes = FREENECT_DEPTH_10BIT_SIZE;
            msg.depth_data_format = KINECT_DEPTH_DATA_T_DEPTH_10BIT;
            break;
        case FREENECT_DEPTH_11BIT_PACKED:
            msg.depth_data_nbytes = FREENECT_DEPTH_11BIT_PACKED_SIZE;
            msg.depth_data_format = KINECT_DEPTH_DATA_T_DEPTH_11BIT_PACKED;
            break;
        case FREENECT_DEPTH_10BIT_PACKED:
            msg.depth_data_nbytes = FREENECT_DEPTH_10BIT_PACKED_SIZE;
            msg.depth_data_format = KINECT_DEPTH_DATA_T_DEPTH_10BIT_PACKED;
            break;
        default:
            msg.depth_data_nbytes = 0;
            msg.depth_data_format = 0; // TODO spew warning
            break;
    }

    msg.depth_data = (uint8_t*) malloc(msg.depth_data_nbytes);
    memcpy(msg.depth_data, data, msg.depth_data_nbytes);

    populate_status_t(state, &msg.status, msg.timestamp);
    
    kinect_depth_data_t_publish(state->lcm, state->depth_channel, &msg);

    dbg("published depth data\n");
}

void image_cb(freenect_device *dev, void *data, uint32_t timestamp)
{
    state_t* state = (state_t*) freenect_get_user(dev);

    kinect_image_data_t msg;
    msg.timestamp = _timestamp_now();
    // TODO better time synchronization

    switch(state->current_image_format) {
        case FREENECT_VIDEO_RGB:
            msg.image_data_nbytes =  FREENECT_VIDEO_RGB_SIZE;
            msg.image_data_format = KINECT_IMAGE_DATA_T_VIDEO_RGB;
            break;
        case FREENECT_VIDEO_BAYER:
            msg.image_data_nbytes =  FREENECT_VIDEO_BAYER_SIZE;
            msg.image_data_format = KINECT_IMAGE_DATA_T_VIDEO_BAYER;
            break;
        case FREENECT_VIDEO_YUV_RGB:
            msg.image_data_nbytes =  FREENECT_VIDEO_YUV_RGB_SIZE;
            msg.image_data_format = KINECT_IMAGE_DATA_T_VIDEO_YUV_RGB;
            break;
        case FREENECT_VIDEO_YUV_RAW:
            msg.image_data_nbytes =  FREENECT_VIDEO_YUV_RAW_SIZE;
            msg.image_data_format = KINECT_IMAGE_DATA_T_VIDEO_YUV_RAW;
            break;
        case FREENECT_VIDEO_IR_8BIT:
            msg.image_data_nbytes =  FREENECT_VIDEO_IR_8BIT_SIZE;
            msg.image_data_format = KINECT_IMAGE_DATA_T_VIDEO_IR_8BIT;
            break;
        case FREENECT_VIDEO_IR_10BIT:
            msg.image_data_nbytes =  FREENECT_VIDEO_IR_10BIT_SIZE;
            msg.image_data_format = KINECT_IMAGE_DATA_T_VIDEO_IR_10BIT;
            break;
        case FREENECT_VIDEO_IR_10BIT_PACKED:
            msg.image_data_nbytes =  FREENECT_VIDEO_IR_10BIT_PACKED_SIZE;
            msg.image_data_format = KINECT_IMAGE_DATA_T_VIDEO_IR_10BIT_PACKED;
            break;
        default:
            msg.image_data_nbytes = 0; // TODO spew warning
            msg.image_data_format = -1;
            break;
    }
    msg.image_data = (uint8_t*) malloc(msg.image_data_nbytes);
    memcpy(msg.image_data, data, msg.image_data_nbytes);

    populate_status_t(state, &msg.status, msg.timestamp);
    
    kinect_image_data_t_publish(state->lcm, state->image_channel, &msg);

    dbg("published image data\n");
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
    printf("looping\n");

	while (!state->die && freenect_process_events(state->f_ctx) >= 0) {
        printf("loop\n");
		if (state->requested_image_format != state->current_image_format) {
			freenect_stop_video(state->f_dev);
			freenect_set_video_format(state->f_dev, state->requested_image_format);
			freenect_start_video(state->f_dev);
			state->current_image_format = state->requested_image_format;
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

int main(int argc, char **argv)
{
	int res;

    state_t *state = (state_t*)calloc(1, sizeof(state_t));
    state->freenect_thread = NULL;
    state->die = 0;
    state->f_ctx = NULL;
    state->f_dev = NULL;
    state->freenect_angle = 0;
    state->freenect_led = 0;

    state->requested_image_format = FREENECT_VIDEO_RGB;
    state->requested_depth_format = FREENECT_DEPTH_11BIT;
    state->requested_led = LED_RED;
    state->current_image_format = state->requested_image_format;
    state->current_depth_format = state->requested_depth_format;
    state->current_led = state->requested_led;

	state->depth_data = (uint8_t*) malloc(640*480*sizeof(float));
	state->image_data   = (uint8_t*) malloc(640*480*4);

    state->image_channel = strdup("KINECT_IMAGE");
    state->depth_channel = strdup("KINECT_DEPTH");

	if(freenect_init(&state->f_ctx, NULL) < 0) {
		printf("freenect_init() failed\n");
		return 1;
	}

    state->lcm = lcm_create(NULL);
    if(!state->lcm) {
        fprintf(stderr, "Unable to initialize LCM\n");
        return 1;
    }

	freenect_set_log_level(state->f_ctx, FREENECT_LOG_INFO);

	int num_devices = freenect_num_devices(state->f_ctx);
	printf("Number of devices found: %d\n", num_devices);

	int user_device_number = 0;
	if (argc > 1)
		user_device_number = atoi(argv[1]);

	if (num_devices < 1)
		return 1;

	if (freenect_open_device(state->f_ctx, &state->f_dev, user_device_number) < 0) {
		printf("Could not open device\n");
		return 1;
	}

    freenect_set_user(state->f_dev, state);

    g_thread_init(NULL);

    GError *thread_err = NULL;
    state->freenect_thread = g_thread_create(freenect_threadfunc, state, TRUE, &thread_err);
    if(thread_err) {
        fprintf(stderr, "Error creating thread: %s\n", thread_err->message);
        return 1;
    }

    // subscribe to kinect command messages...

    while(!state->die) {
        lcm_handle(state->lcm);
    }

    free(state->image_channel);
    free(state->depth_channel);
    free(state);

	return 0;
}
