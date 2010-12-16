#include <zlib.h>

#include <lcm/lcm.h>

#include <bot_core/bot_core.h>
#include <bot_vis/bot_vis.h>

#include <lcmtypes/kinect_frame_t.h>

#include "jpeg-utils-ijg.h"

typedef struct _KinectRenderer {
    BotRenderer renderer;
    BotGtkParamWidget *pw;
    BotViewer   *viewer;
    lcm_t     *lcm;

    kinect_frame_t* msg;

    int width;
    int height;

    // raw disparity
    uint16_t* disparity;
    int need_to_recompute_frame_data;

    uint8_t* uncompress_buffer;
    int uncompress_buffer_size;

    uint8_t* rgb_data;
} KinectRenderer;

static void
recompute_frame_data(KinectRenderer* self);

#if 0
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
#endif

static void
recompute_frame_data(KinectRenderer* self)
{
    if(!self->msg) {
        self->need_to_recompute_frame_data = 0;
        return;
    }

    int npixels = self->width * self->height;

    const uint8_t* depth_data = self->msg->depth.depth_data;

    if(self->msg->depth.compression != KINECT_DEPTH_DATA_T_COMPRESSION_NONE) {
        if(self->msg->depth.uncompressed_size > self->uncompress_buffer_size) {
            self->uncompress_buffer_size = self->msg->depth.uncompressed_size;
            self->uncompress_buffer = (uint8_t*) realloc(self->uncompress_buffer, self->uncompress_buffer_size);
        }
        unsigned long dlen = self->msg->depth.uncompressed_size;
        int status = uncompress(self->uncompress_buffer, &dlen, 
                self->msg->depth.depth_data, self->msg->depth.depth_data_nbytes);
        if(status != Z_OK) {
            return;
        }
        depth_data = self->uncompress_buffer;
    }

    switch(self->msg->depth.depth_data_format) {
        case KINECT_DEPTH_DATA_T_DEPTH_11BIT:
            if(G_BYTE_ORDER == G_LITTLE_ENDIAN) {
                int16_t* rdd = (int16_t*) depth_data;
                int i;
                for(i=0; i<npixels; i++) {
                    int d = rdd[i];
                    self->disparity[i] = d;
                }
            } else {
                fprintf(stderr, "Big endian systems not supported\n");
            }
            break;
        case KINECT_DEPTH_DATA_T_DEPTH_10BIT:
            fprintf(stderr, "10-bit depth data not supported\n");
            break;
#if 0
        case KINECT_DEPTH_DATA_T_DEPTH_11BIT_PACKED:
            if(G_BYTE_ORDER == G_LITTLE_ENDIAN) {
                int mask = (1 << 11) - 1;
                uint32_t buffer = 0;
                int bitsIn = 0;
                uint8_t* raw = (uint8_t*) depth_data;

                for(int i=0; i<npixels; i++) {
                    while (bitsIn < 11) {
                        buffer = (buffer << 8) | *(raw++);
                        bitsIn += 8;
                    }
                    bitsIn -= 11;
                    self->disparity[i] = (buffer >> bitsIn) & mask;
                }
            } else {
                fprintf(stderr, "Big endian systems not supported\n");
            }
            break;
        case KINECT_DEPTH_DATA_T_DEPTH_10BIT_PACKED:
            fprintf(stderr, "10-bit packed depth data not supported\n");
            break;
#endif
        default:
            break;
    }
}

static void 
on_kinect_frame (const lcm_recv_buf_t *rbuf, const char *channel,
        const kinect_frame_t *msg, void *user_data )
{
    KinectRenderer *self = (KinectRenderer*) user_data;

    printf("frame\n");

    if(self->msg)
        kinect_frame_t_destroy(self->msg);
    self->msg = kinect_frame_t_copy(msg);

    // TODO check width, height

    if(msg->image.image_data_format == KINECT_IMAGE_DATA_T_VIDEO_RGB) {
        memcpy(self->rgb_data, msg->image.image_data, 
                self->width * self->height * 3);
    } else if(msg->image.image_data_format == KINECT_IMAGE_DATA_T_VIDEO_RGB_JPEG) {
        jpegijg_decompress_8u_rgb(msg->image.image_data, msg->image.image_data_nbytes,
                self->rgb_data, self->width, self->height, self->width * 3);
    }

    self->need_to_recompute_frame_data = 1;

    bot_viewer_request_redraw(self->viewer);
}

static void on_param_widget_changed (BotGtkParamWidget *pw, const char *name, void *user)
{
    KinectRenderer *self = (KinectRenderer*) user;

    // TODO

    bot_viewer_request_redraw(self->viewer);
}

static inline void
_matrix_multiply (const double *a, int a_nrows, int a_ncols,
        const double *b, int b_nrows, int b_ncols,
        double *result)
{
    int i, j, r;
    assert (a_ncols == b_nrows);
    for (i=0; i<a_nrows; i++) {
        for (j=0; j<b_ncols; j++) {
            double acc = 0;
            for (r=0; r<a_ncols; r++) {
                acc += a[i*a_ncols + r] * b[r*b_ncols + j];
            }
            result[i*b_ncols + j] = acc;
        }
    }
}

static inline void
_matrix_vector_multiply_3x4_4d (const double m[12], const double v[4],
        double result[3])
{
    result[0] = m[0]*v[0] + m[1]*v[1] + m[2] *v[2] + m[3] *v[3];
    result[1] = m[4]*v[0] + m[5]*v[1] + m[6] *v[2] + m[7] *v[3];
    result[2] = m[8]*v[0] + m[9]*v[1] + m[10]*v[2] + m[11]*v[3];
}

static inline void
_matrix_transpose_4x4d (const double m[16], double result[16])
{
    result[0] = m[0];
    result[1] = m[4];
    result[2] = m[8];
    result[3] = m[12];
    result[4] = m[1];
    result[5] = m[5];
    result[6] = m[9];
    result[7] = m[13];
    result[8] = m[2];
    result[9] = m[6];
    result[10] = m[10];
    result[11] = m[14];
    result[12] = m[3];
    result[13] = m[7];
    result[14] = m[11];
    result[15] = m[15];
}

static void _draw(BotViewer *viewer, BotRenderer *renderer)
{
    KinectRenderer *self = (KinectRenderer*) renderer->user;
    if(!self->msg)
        return;

    if(self->need_to_recompute_frame_data)
        recompute_frame_data(self);

    // rotate so that X is forward and Z is up
    glRotatef(-90, 0, 0, 1);
    glRotatef(-90, 1, 0, 0);

    float so = 1101.5254;
    float b = 0.07710;
    float cx = 317.80483131;
    float cy = 243.16590456;
    float f = 579.97821782;

    double depth_to_depth_xyz[] = {
        1, 0, 0, -cx,
        0, 1, 0, -cy,
        0, 0, 0, f,
        0, 0, 1/b, 0
    };
    double depth_to_rgb_rot_trans[] = {
        0.999997, -0.002517, -0.000720, -0.025318, 
        0.002518, 0.999995, 0.001772, 0.000415, 
        0.000716, -0.001774, 0.999998, -0.011306,
        0, 0, 0, 1
    };
    double cx_rgb = 322.60598443;
    double cy_rgb = 262.56367112;
    double k1_rgb = 0.17730118;
    double k2_rgb = -0.35126598;
    double f_rgb = 518.12282951;
//    double p1_rgb = 0;
//    double p2_rgb = 0;
//    double k3_rgb = 0;

    double rgb_k[] = {
        f_rgb, 0, cx_rgb, 0,
        0, f_rgb, cy_rgb, 0,
        0, 0, 1, 0
    };
    double depth_to_rgb_xyz[16];
    _matrix_multiply(depth_to_rgb_rot_trans, 4, 4, 
            depth_to_depth_xyz, 4, 4, depth_to_rgb_xyz);
    double depth_to_rgb_uvd[12];
    _matrix_multiply(rgb_k, 3, 4, 
            depth_to_rgb_xyz, 4, 4, 
            depth_to_rgb_uvd);

    double depth_to_depth_xyz_trans[16];
    _matrix_transpose_4x4d(depth_to_depth_xyz, depth_to_depth_xyz_trans);
    glPushMatrix();
    glMultMatrixd(depth_to_depth_xyz_trans);
    glBegin(GL_POINTS);
    glColor3f(0, 0, 0);
    for(int u=0; u<self->width; u++) {
        for(int v=0; v<self->height; v++) {
            uint16_t disparity = self->disparity[v*self->width+u];
            float d = 0.125 * (so - disparity);

            double uvd_depth[4] = { u, v, d, 1 };
            double uvd_rgb[3];
            _matrix_vector_multiply_3x4_4d(depth_to_rgb_uvd, uvd_depth, uvd_rgb);

            double u_rgb_undist = uvd_rgb[0] / uvd_rgb[2];
            double v_rgb_undist = uvd_rgb[1] / uvd_rgb[2];

            // compute distorted pixel coordinates
            double du_rgb = (u_rgb_undist - cx_rgb) / f_rgb;
            double dv_rgb = (v_rgb_undist - cy_rgb) / f_rgb;
            double rad_rgb_2 = du_rgb*du_rgb + dv_rgb*dv_rgb;
            double rad_rgb_4 = rad_rgb_2*rad_rgb_2;
            double s = (1 + k1_rgb * rad_rgb_2 + k2_rgb * rad_rgb_4) * f_rgb;
            int u_rgb = du_rgb * s + cx_rgb + 0.5;
            int v_rgb = dv_rgb * s + cy_rgb + 0.5;

            uint8_t r, g, b;
            if(u_rgb >= self->width || u_rgb < 0 || v_rgb >= self->height || v_rgb < 0) {
                r = g = b = 0;
            } else {
                r = self->rgb_data[v_rgb*self->width*3 + u_rgb*3 + 0];
                g = self->rgb_data[v_rgb*self->width*3 + u_rgb*3 + 1];
                b = self->rgb_data[v_rgb*self->width*3 + u_rgb*3 + 2];
            }

            glColor3f(r / 255.0, g / 255.0, b / 255.0);

            glVertex3f(u, v, d);
        }
    }
    glEnd();
    glPopMatrix();
}

static void _free(BotRenderer *renderer)
{
    KinectRenderer *self = (KinectRenderer*) renderer;

    if(self->msg)
        kinect_frame_t_destroy(self->msg);

    free(self->uncompress_buffer);
    self->uncompress_buffer_size = 0;

    free(self->disparity);
    free(self->rgb_data);

    free(self);
}

void 
kinect_add_renderer_to_viewer(BotViewer* viewer, lcm_t* lcm, int priority)
{
    KinectRenderer *self = (KinectRenderer*) calloc(1, sizeof(KinectRenderer));

    self->need_to_recompute_frame_data = 0;
    self->width = 640;
    self->height = 480;
    self->disparity = (uint16_t*) malloc(self->width * self->height * sizeof(uint16_t));
    self->rgb_data = (uint8_t*) malloc(self->width * self->height * 3);

    self->msg = NULL;

    BotRenderer *renderer = &self->renderer;

    self->lcm = lcm;
    self->viewer = viewer;
    self->pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());

    self->uncompress_buffer = NULL;
    self->uncompress_buffer_size = 0;

    renderer->draw = _draw;
    renderer->destroy = _free;
    renderer->name = "Kinect";
    renderer->widget = GTK_WIDGET(self->pw);
    renderer->enabled = 1;
    renderer->user = self;

    g_signal_connect (G_OBJECT (self->pw), "changed",
                      G_CALLBACK (on_param_widget_changed), self);

    kinect_frame_t_subscribe(self->lcm, "KINECT_FRAME", on_kinect_frame, self);

    bot_viewer_add_renderer(viewer, renderer, priority);
}
