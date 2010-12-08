#include <lcm/lcm.h>

#include <bot_core/bot_core.h>
#include <bot_vis/bot_vis.h>

#include <lcmtypes/kinect_image_data_t.h>
#include <lcmtypes/kinect_depth_data_t.h>

typedef struct _KinectRenderer {
    BotRenderer renderer;
    BotGtkParamWidget *pw;
    BotViewer   *viewer;
    lcm_t     *lcm;

    kinect_image_data_t* image_msg;
    kinect_depth_data_t* depth_msg;

    int width;
    int height;

    // depth data, in meters.
    float* depth_data;
    int need_to_recompute_depth_data;
} KinectRenderer;

static void
recompute_depth_data(KinectRenderer* self);

static void
recompute_depth_data(KinectRenderer* self)
{
    if(!self->depth_msg) {
        self->need_to_recompute_depth_data = 0;
        return;
    }

//    int npixels = self->width * self->height;

    switch(self->depth_msg->depth_data_format) {
        case KINECT_DEPTH_DATA_T_DEPTH_11BIT:
            // TODO
            break;
        case KINECT_DEPTH_DATA_T_DEPTH_10BIT:
            // TODO
            break;
        case KINECT_DEPTH_DATA_T_DEPTH_11BIT_PACKED:
            // TODO
            break;
        case KINECT_DEPTH_DATA_T_DEPTH_10BIT_PACKED:
            // TODO
            break;
        default:
            break;
    }
}

#if 0
int window;
GLuint gl_rgb_tex;
int mx=-1,my=-1;        // Prevous mouse coordinates
int rotangles[2] = {0}; // Panning angles
float zoom = 1;         // zoom factor
int color = 1;          // Use the RGB texture or just draw it as color

// Do the projection from u,v,depth to X,Y,Z directly in an opengl matrix
// These numbers come from a combination of the ros kinect_node wiki, and
// nicolas burrus' posts.
void LoadVertexMatrix()
{
	float f = 590.0f;
	float a = -0.0030711f;
	float b = 3.3309495f;
	float cx = 340.0f;
	float cy = 240.0f;
	GLfloat mat[16] = {
		1/f,     0,  0, 0,
		0,    -1/f,  0, 0,
		0,       0,  0, a,
		-cx/f,cy/f, -1, b
	};
	glMultMatrixf(mat);
}


// This matrix comes from a combination of nicolas burrus's calibration post
// and some python code I haven't documented yet.
void LoadRGBMatrix()
{
	float mat[16] = {
		  5.34866271e+02,   3.89654806e+00,   0.00000000e+00,   1.74704200e-02,
		 -4.70724694e+00,  -5.28843603e+02,   0.00000000e+00,  -1.22753400e-02,
		 -3.19670762e+02,  -2.60999685e+02,   0.00000000e+00,  -9.99772000e-01,
		 -6.98445586e+00,   3.31139785e+00,   0.00000000e+00,   1.09167360e-02
	};
	glMultMatrixf(mat);
}

void mouseMoved(int x, int y)
{
	if (mx>=0 && my>=0) {
		rotangles[0] += y-my;
		rotangles[1] += x-mx;
	}
	mx = x;
	my = y;
}

void mousePress(int button, int state, int x, int y)
{
	if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
		mx = x;
		my = y;
	}
	if (button == GLUT_LEFT_BUTTON && state == GLUT_UP) {
		mx = -1;
		my = -1;
	}
}

void DrawGLScene()
{
	short *depth = 0;
	char *rgb = 0;
	uint32_t ts;
	if (freenect_sync_get_depth((void**)&depth, &ts) < 0)
		return;
	if (freenect_sync_get_rgb((void**)&rgb, &ts) < 0) {
		free(depth);
		return;
	}

	static unsigned int indices[480][640];
	static short xyz[480][640][3];
	int i,j;
	for (i = 0; i < 480; i++) {
		for (j = 0; j < 640; j++) {
			xyz[i][j][0] = j;
			xyz[i][j][1] = i;
			xyz[i][j][2] = depth[i*640+j];
			indices[i][j] = i*640+j;
		}
	}

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	glPushMatrix();
	glScalef(zoom,zoom,1);
	glTranslatef(0,0,-3.5);
	glRotatef(rotangles[0], 1,0,0);
	glRotatef(rotangles[1], 0,1,0);
	glTranslatef(0,0,1.5);

	LoadVertexMatrix();

	// Set the projection from the XYZ to the texture image
	glMatrixMode(GL_TEXTURE);
	glLoadIdentity();
	glScalef(1/640.0f,1/480.0f,1);
	LoadRGBMatrix();
	LoadVertexMatrix();
	glMatrixMode(GL_MODELVIEW);

	glPointSize(1);

	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_SHORT, 0, xyz);
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	glTexCoordPointer(3, GL_SHORT, 0, xyz);

	if (color)
		glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, gl_rgb_tex);
	glTexImage2D(GL_TEXTURE_2D, 0, 3, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, rgb);

	glPointSize(2.0f);
	glDrawElements(GL_POINTS, 640*480, GL_UNSIGNED_INT, indices);
	glPopMatrix();
	glDisable(GL_TEXTURE_2D);

	free(rgb);
	free(depth);

	glutSwapBuffers();
}
#endif

static void 
on_kinect_image (const lcm_recv_buf_t *rbuf, const char *channel,
        const kinect_image_data_t *msg, void *user_data )
{
    KinectRenderer *self = (KinectRenderer*) user_data;

    if(self->image_msg)
        kinect_image_data_t_destroy(self->image_msg);
    self->image_msg = kinect_image_data_t_copy(self->image_msg);

    bot_viewer_request_redraw( self->viewer );
}

static void 
on_kinect_depth (const lcm_recv_buf_t *rbuf, const char *channel,
        const kinect_depth_data_t *msg, void *user_data )
{
    KinectRenderer *self = (KinectRenderer*) user_data;

    if(self->depth_msg)
        kinect_depth_data_t_destroy(self->depth_msg);
    self->depth_msg = kinect_depth_data_t_copy(self->depth_msg);

    self->need_to_recompute_depth_data = 1;

    bot_viewer_request_redraw( self->viewer );
}

static void on_param_widget_changed (BotGtkParamWidget *pw, const char *name, void *user)
{
    KinectRenderer *self = (KinectRenderer*) user;

    // TODO

    bot_viewer_request_redraw(self->viewer);
}

static void _draw(BotViewer *viewer, BotRenderer *renderer)
{
    KinectRenderer *self = (KinectRenderer*) renderer->user;

    if(self->need_to_recompute_depth_data)
        recompute_depth_data(self);

    // TODO
}

static void _free(BotRenderer *renderer)
{
    KinectRenderer *self = (KinectRenderer*) renderer;

    if(self->image_msg)
        kinect_image_data_t_destroy(self->image_msg);
    if(self->depth_msg)
        kinect_depth_data_t_destroy(self->depth_msg);

    free(self->depth_data);

    free(self);
}

void 
kinect_add_renderer_to_viewer(BotViewer* viewer, lcm_t* lcm, int priority)
{
    KinectRenderer *self = (KinectRenderer*) calloc(1, sizeof(KinectRenderer));

    self->need_to_recompute_depth_data = 0;
    self->width = 640;
    self->height = 480;
    self->depth_data = (float*) malloc(self->width * self->height * sizeof(float));

    self->image_msg = NULL;
    self->depth_msg = NULL;

    BotRenderer *renderer = &self->renderer;

    self->lcm = lcm;
    self->viewer = viewer;
    self->pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());

    renderer->draw = _draw;
    renderer->destroy = _free;
    renderer->name = "Kinect";
    renderer->widget = GTK_WIDGET(self->pw);
    renderer->enabled = 1;
    renderer->user = self;

    g_signal_connect (G_OBJECT (self->pw), "changed",
                      G_CALLBACK (on_param_widget_changed), self);

    kinect_image_data_t_subscribe(self->lcm, "KINECT_IMAGE", on_kinect_image, self);
    kinect_depth_data_t_subscribe(self->lcm, "KINECT_DEPTH", on_kinect_depth, self);

    bot_viewer_add_renderer(viewer, renderer, priority);

//    GtkWidget *clear_button = gtk_button_new_with_label("Clear All");
//    gtk_box_pack_start(GTK_BOX(renderer->widget), clear_button, FALSE, FALSE, 0);
//    g_signal_connect(G_OBJECT(clear_button), "clicked", G_CALLBACK(on_clear_button), self);
//    gtk_widget_show_all(renderer->widget);
}
