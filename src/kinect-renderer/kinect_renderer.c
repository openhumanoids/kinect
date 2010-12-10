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
    float* xyz;
    int need_to_recompute_frame_data;

    int depth_lut_size;
    float* depth_lut_11bit;
} KinectRenderer;

static void
recompute_frame_data_data(KinectRenderer* self);

/** Given an array of colors, a palette is created that linearly interpolates through all the colors. **/
static void color_util_build_color_table(double color_palette[][3], int palette_size, float lut[][3], int lut_size)
{
    for (int idx = 0; idx < lut_size; idx++) {
        double znorm = ((double) idx) / lut_size;

        int color_index = (palette_size - 1) * znorm;
        double alpha = (palette_size - 1) * znorm - color_index;
        
        for (int i = 0; i < 3; i++) {
            lut[idx][i] = color_palette[color_index][i] * (1.0 - alpha) + color_palette[color_index+1][i]*alpha;
        }    
    }
}

#define JET_COLORS_LUT_SIZE 1024
static float jet_colors[JET_COLORS_LUT_SIZE][3];
static int jet_colors_initialized = 0;

static void init_color_table_jet()
{
    double jet[][3] = {{ 0,   0,   1 },
                       { 0,  .5,  .5 },
                       { .8, .8,   0 },
                       { 1,   0,   0 }};

    color_util_build_color_table(jet, sizeof(jet)/(sizeof(double)*3), jet_colors, JET_COLORS_LUT_SIZE);
    jet_colors_initialized = 1;
}

static inline float *color_util_jet(double v)
{
    if (!jet_colors_initialized)
        init_color_table_jet();

    v = fmax(0, v);
    v = fmin(1, v);

    int idx = (JET_COLORS_LUT_SIZE - 1) * v;
    return jet_colors[idx];
}

static void
compute_lookup_tables(KinectRenderer* self)
{
    int raw_disparity;
    self->depth_lut_size = 2048;
    self->depth_lut_11bit = (float*) malloc(self->depth_lut_size * sizeof(float));

    for(raw_disparity=0; raw_disparity<self->depth_lut_size; raw_disparity++) {
        self->depth_lut_11bit[raw_disparity] = 0.1236 * tan(raw_disparity / 2842.5 + 1.1863);
    }

    // TODO replace these with calibrated values
    double fx_d = 5.9421434211923247e+02;
    double fy_d = 5.9104053696870778e+02;
    double cx_d = 3.3930780975300314e+02;
    double cy_d = 2.4273913761751615e+02;
//    double k1_d = -2.6386489753128833e-01;
//    double k2_d = 9.9966832163729757e-01;
//    double p1_d = -7.6275862143610667e-04;
//    double p2_d = 5.0350940090814270e-03;
//    double k3_d = -1.3053628089976321e+00;

    int pind = 0;
    for(int v=0; v<self->height; v++) {
        for(int u=0; u<self->width; u++) {
            self->xyz[pind*3+0] = (u - cx_d) / fx_d;
            self->xyz[pind*3+1] = (v - cy_d) / fy_d;
            self->xyz[pind*3+2] = 1;
            pind++;
        }
    }

}

static void
recompute_frame_data_data(KinectRenderer* self)
{
    if(!self->depth_msg) {
        self->need_to_recompute_frame_data = 0;
        return;
    }

    int npixels = self->width * self->height;

    switch(self->depth_msg->depth_data_format) {
        case KINECT_DEPTH_DATA_T_DEPTH_11BIT:
            if(G_BYTE_ORDER == G_LITTLE_ENDIAN) {
                int16_t* rdd = (int16_t*) self->depth_msg->depth_data;
                int i;
                for(i=0; i<npixels; i++) {
                    int d = rdd[i];
                    if(d >= 0 && d < self->depth_lut_size)
                        self->xyz[i*3+2] = self->depth_lut_11bit[d];
                    else
                        self->xyz[i*3+2] = 0;
                }
            } else {
                fprintf(stderr, "Big endian systems not supported\n");
            }
            break;
        case KINECT_DEPTH_DATA_T_DEPTH_10BIT:
            fprintf(stderr, "10-bit depth data not supported\n");
            break;
        case KINECT_DEPTH_DATA_T_DEPTH_11BIT_PACKED:
            // TODO
            break;
        case KINECT_DEPTH_DATA_T_DEPTH_10BIT_PACKED:
            fprintf(stderr, "10-bit packed depth data not supported\n");
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
    self->image_msg = kinect_image_data_t_copy(msg);

    bot_viewer_request_redraw( self->viewer );
}

static void 
on_kinect_depth (const lcm_recv_buf_t *rbuf, const char *channel,
        const kinect_depth_data_t *msg, void *user_data )
{
    KinectRenderer *self = (KinectRenderer*) user_data;

    if(self->depth_msg)
        kinect_depth_data_t_destroy(self->depth_msg);
    self->depth_msg = kinect_depth_data_t_copy(msg);

    self->need_to_recompute_frame_data = 1;

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
    if(!self->depth_msg)
        return;

    if(self->need_to_recompute_frame_data)
        recompute_frame_data_data(self);

    // rotate so that X is forward and Z is up
    glRotatef(-90, 0, 0, 1);
    glRotatef(-90, 1, 0, 0);

    glBegin(GL_POINTS);
    glColor3f(0, 0, 0);
    for(int pind=0; pind<self->width*self->height; pind++) {
        float x = self->xyz[pind*3+0] * self->xyz[pind*3+2];
        float y = self->xyz[pind*3+1] * self->xyz[pind*3+2];
        float z = self->xyz[pind*3+2];
//        float* color = color_util_jet((y + 1.5) / 2);
//        glColor3fv(color);
        glVertex3f(x, y, z);
    }
    glEnd();
}

static void _free(BotRenderer *renderer)
{
    KinectRenderer *self = (KinectRenderer*) renderer;

    if(self->image_msg)
        kinect_image_data_t_destroy(self->image_msg);
    if(self->depth_msg)
        kinect_depth_data_t_destroy(self->depth_msg);

    free(self->xyz);

    free(self);
}

void 
kinect_add_renderer_to_viewer(BotViewer* viewer, lcm_t* lcm, int priority)
{
    KinectRenderer *self = (KinectRenderer*) calloc(1, sizeof(KinectRenderer));

    self->need_to_recompute_frame_data = 0;
    self->width = 640;
    self->height = 480;
    self->xyz = (float*) malloc(self->width * self->height * 3 * sizeof(float));

    self->image_msg = NULL;
    self->depth_msg = NULL;

    self->depth_lut_size = 0;
    self->depth_lut_11bit = NULL;

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

    compute_lookup_tables(self);

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
