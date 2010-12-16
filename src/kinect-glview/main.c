#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <poll.h>

#include <zlib.h>

#include <lcm/lcm.h>
#include <lcmtypes/kinect_depth_data_t.h>
#include <lcmtypes/kinect_image_data_t.h>

#if defined(__APPLE__)
#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include <math.h>

#include "jpeg-utils-ijg.h"

int window;

int width = 640;
int height = 480;

uint8_t* depth_img;
uint8_t* depth_uncompress_buffer;
uint8_t* rgb;

GLuint gl_depth_tex;
GLuint gl_rgb_tex;

int got_rgb = 0;
int got_depth = 0;

lcm_t* lcm = NULL;

void DrawGLScene()
{
    struct pollfd pfd = { lcm_get_fileno(lcm), POLLIN, 0 };
    int status = poll (&pfd, 1, 10);
    if (status > 0) {
        lcm_handle(lcm);
    }

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	glEnable(GL_TEXTURE_2D);

	glBindTexture(GL_TEXTURE_2D, gl_depth_tex);
	glTexImage2D(GL_TEXTURE_2D, 0, 3, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, depth_img);

	glBegin(GL_TRIANGLE_FAN);
	glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
	glTexCoord2f(0, 0); glVertex3f(0,0,0);
	glTexCoord2f(1, 0); glVertex3f(640,0,0);
	glTexCoord2f(1, 1); glVertex3f(640,480,0);
	glTexCoord2f(0, 1); glVertex3f(0,480,0);
	glEnd();

	glBindTexture(GL_TEXTURE_2D, gl_rgb_tex);
//	if (current_format == FREENECT_VIDEO_RGB || current_format == FREENECT_VIDEO_YUV_RGB)
    glTexImage2D(GL_TEXTURE_2D, 0, 3, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, rgb);
//	else
//		glTexImage2D(GL_TEXTURE_2D, 0, 1, 640, 480, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, rgb_front+640*4);

	glBegin(GL_TRIANGLE_FAN);
	glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
	glTexCoord2f(0, 0); glVertex3f(640,0,0);
	glTexCoord2f(1, 0); glVertex3f(1280,0,0);
	glTexCoord2f(1, 1); glVertex3f(1280,480,0);
	glTexCoord2f(0, 1); glVertex3f(640,480,0);
	glEnd();

	glutSwapBuffers();
}

void keyPressed(unsigned char key, int x, int y)
{
	if (key == 27) {
		glutDestroyWindow(window);
	}
}

void ReSizeGLScene(int Width, int Height)
{
	glViewport(0,0,Width,Height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho (0, 1280, 480, 0, -1.0f, 1.0f);
	glMatrixMode(GL_MODELVIEW);
}

void InitGL(int Width, int Height)
{
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClearDepth(1.0);
	glDepthFunc(GL_LESS);
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glShadeModel(GL_SMOOTH);
	glGenTextures(1, &gl_depth_tex);
	glBindTexture(GL_TEXTURE_2D, gl_depth_tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glGenTextures(1, &gl_rgb_tex);
	glBindTexture(GL_TEXTURE_2D, gl_rgb_tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	ReSizeGLScene(Width, Height);
}

uint16_t t_gamma[2048];

static void
on_image_data(const lcm_recv_buf_t* lcm, const char* channel, const kinect_image_data_t* msg, void* user)
{
    // TODO check image width, height

    if(msg->image_data_format == KINECT_IMAGE_DATA_T_VIDEO_RGB) {
        memcpy(rgb, msg->image_data, width * height * 3);
    } else if(msg->image_data_format == KINECT_IMAGE_DATA_T_VIDEO_RGB_JPEG) {
        jpegijg_decompress_8u_rgb (msg->image_data, msg->image_data_nbytes,
                rgb, width, height, width * 3);
    }
    got_rgb++;
}

static void
on_depth_data(const lcm_recv_buf_t* lcm, const char* channel, const kinect_depth_data_t* msg, void* user)
{
    int i;
    const uint16_t* depth = NULL;
    if(msg->compression == KINECT_DEPTH_DATA_T_COMPRESSION_NONE) {
        depth = (uint16_t*) msg->depth_data;
    } else if (msg->compression == KINECT_DEPTH_DATA_T_COMPRESSION_ZLIB) {
        unsigned long dlen = msg->uncompressed_size;
        uncompress(depth_uncompress_buffer, &dlen, msg->depth_data, msg->depth_data_nbytes);
        depth = (uint16_t*) depth_uncompress_buffer;
    }

    int npixels = width * height;
	for (i=0; i<npixels; i++) {
#if 0
        int max = 2048;
        int min = 800;
        int p = (int)(255 * (depth[i] - min) / (max - min));
        depth_img[i*3 + 0] = p;
        depth_img[i*3 + 1] = p;
        depth_img[i*3 + 2] = p;
#else
		int pval = t_gamma[depth[i]];
		int lb = pval & 0xff;
		switch (pval>>8) {
			case 0:
				depth_img[3*i+0] = 255;
				depth_img[3*i+1] = 255-lb;
				depth_img[3*i+2] = 255-lb;
				break;
			case 1:
				depth_img[3*i+0] = 255;
				depth_img[3*i+1] = lb;
				depth_img[3*i+2] = 0;
				break;
			case 2:
				depth_img[3*i+0] = 255-lb;
				depth_img[3*i+1] = 255;
				depth_img[3*i+2] = 0;
				break;
			case 3:
				depth_img[3*i+0] = 0;
				depth_img[3*i+1] = 255;
				depth_img[3*i+2] = lb;
				break;
			case 4:
				depth_img[3*i+0] = 0;
				depth_img[3*i+1] = 255-lb;
				depth_img[3*i+2] = 255;
				break;
			case 5:
				depth_img[3*i+0] = 0;
				depth_img[3*i+1] = 0;
				depth_img[3*i+2] = 255-lb;
				break;
			default:
				depth_img[3*i+0] = 0;
				depth_img[3*i+1] = 0;
				depth_img[3*i+2] = 0;
				break;
		}
#endif
	}
	got_depth++;
}

int main(int argc, char **argv)
{
	int res;

    width = 640;
    height = 480;
    int npixels = width*height;

	depth_img = malloc(npixels*3);
	depth_uncompress_buffer = malloc(npixels*sizeof(uint16_t));
	rgb = malloc(npixels*3);

	int i;
	for (i=0; i<2048; i++) {
		float v = i/2048.0;
		v = powf(v, 3)* 6;
		t_gamma[i] = v*6*256;
	}

	glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);
	glutInitWindowSize(1280, 480);
	glutInitWindowPosition(0, 0);

	window = glutCreateWindow("Kinect LCM viewer");

	glutDisplayFunc(&DrawGLScene);
	glutIdleFunc(&DrawGLScene);
	glutReshapeFunc(&ReSizeGLScene);
	glutKeyboardFunc(&keyPressed);

	InitGL(1280, 480);

    lcm = lcm_create(NULL);

    kinect_image_data_t_subscribe(lcm, "KINECT_IMAGE", on_image_data, NULL);
    kinect_depth_data_t_subscribe(lcm, "KINECT_DEPTH", on_depth_data, NULL);

    glutMainLoop();

    free(depth_img);
    free(depth_uncompress_buffer);
    free(rgb);
    lcm_destroy(lcm);

	return 0;
}
