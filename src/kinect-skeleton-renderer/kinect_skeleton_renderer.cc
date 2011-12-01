#include "kinect_skeleton_renderer.h"
#include <lcm/lcm.h>

#include <bot_core/bot_core.h>
#include <bot_vis/bot_vis.h>
#include <bot_frames/bot_frames.h>

//#include <lcmtypes/kinect_frame_msg_t.h>
//#include <lcmtypes/kinect_point_list_t.h>
#include <lcmtypes/kinect_skeleton_msg_t.h>
#include <lcmtypes/kinect_link_msg_t.h>
#include <kinect/kinect-utils.h>

typedef struct _KinectSkeletonRenderer {
    BotRenderer renderer;
    BotGtkParamWidget *pw;
    BotViewer   *viewer;
    lcm_t     *lcm;
    BotFrames *frames;
    char * kinect_frame;

    kinect_skeleton_msg_t* skeleton_msg;
    //kinect_depth_msg_t* depth_msg;
    //kinect_point_list_t* point_list_msg;
    //kinect_point_list_t* point_list_msg_debug;

    int width, height;

    // raw disparity
    //uint16_t* disparity;


    KinectCalibration* kcal;
} KinectSkeletonRenderer;

// static void 
// on_kinect_frame (const lcm_recv_buf_t *rbuf, const char *channel,
//         const kinect_depth_msg_t *depth_msg, void *user_data )
// {
//     KinectSkeletonRenderer *self = (KinectSkeletonRenderer*) user_data;

//     if(self->depth_msg)
//         kinect_depth_msg_t_destroy(self->depth_msg);
    
//     // Allocate disparity once based on the image size
//     if (depth_msg) { 

//         // If sizes don't match, reallocate and free the previous disparity image
//         if (depth_msg->width != self->width || depth_msg->height != self->height) { 
//             free(self->disparity);
//             self->disparity = 0;
//             printf("disparity width/height different, reallocating\n");
//         }

//         // Allocate 
//         if (!self->disparity) { 
//             self->width = depth_msg->width;
//             self->height = depth_msg->height;
//             self->disparity = (uint16_t*) malloc(depth_msg->width * depth_msg->height * sizeof(uint16_t));
//         }
//     }
        
//     // Copy the data and recompute frame
//     self->depth_msg = kinect_depth_msg_t_copy(depth_msg);
//     memcpy(self->disparity, depth_msg->depth_data, self->width * self->height * sizeof(uint16_t) );
//     bot_viewer_request_redraw(self->viewer);
// }

// static void 
// on_kinect_point_cloud (const lcm_recv_buf_t *rbuf, const char *channel,
//         const kinect_point_list_t *point_list_msg, void *user_data )
// {
//     KinectSkeletonRenderer *self = (KinectSkeletonRenderer*) user_data;
    
//     if(self->point_list_msg)
//         kinect_point_list_t_destroy(self->point_list_msg);
//     self->point_list_msg = kinect_point_list_t_copy(point_list_msg);

//     bot_viewer_request_redraw(self->viewer);

// }


// static void 
// on_kinect_point_cloud_debug (const lcm_recv_buf_t *rbuf, const char *channel,
//         const kinect_point_list_t *point_list_msg, void *user_data )
// {
//     KinectSkeletonRenderer *self = (KinectSkeletonRenderer*) user_data;
    
//     if(self->point_list_msg_debug)
//         kinect_point_list_t_destroy(self->point_list_msg_debug);
//     self->point_list_msg_debug = kinect_point_list_t_copy(point_list_msg);

//     bot_viewer_request_redraw(self->viewer);

// }

static void 
on_skeleton_frame (const lcm_recv_buf_t *rbuf, const char *channel,
		   const kinect_skeleton_msg_t *skeleton_msg, void *user_data )
{
    KinectSkeletonRenderer *self = (KinectSkeletonRenderer*) user_data;

    if(self->skeleton_msg)
        kinect_skeleton_msg_t_destroy(self->skeleton_msg);
    self->skeleton_msg = kinect_skeleton_msg_t_copy(skeleton_msg);

    bot_viewer_request_redraw(self->viewer);
}

static void on_param_widget_changed (BotGtkParamWidget *pw, const char *name, void *user)
{
    KinectSkeletonRenderer *self = (KinectSkeletonRenderer*) user;

    // TODO

    bot_viewer_request_redraw(self->viewer);
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

// Draws an axis-aligned cube at a giveGL_FRONT_AND_BACKn location.
inline void draw_cube(float x, float y, float z, float width) {
    const float hs = 0.5 * width;
    ///*  
    glBegin( GL_TRIANGLE_STRIP );
    // front-left edge
    glVertex3f( x - hs, y + hs, z - hs );
    glVertex3f( x - hs, y - hs, z - hs );
      
    // front-right edge
    glVertex3f( x + hs, y + hs, z - hs );
    glVertex3f( x + hs, y - hs, z - hs );

    // back-right edge
    glVertex3f( x + hs, y + hs, z + hs );
    glVertex3f( x + hs, y - hs, z + hs );
    
    // back-left edge
    glVertex3f( x - hs, y + hs, z + hs );
    glVertex3f( x - hs, y - hs, z + hs );
      
    // front-left edge (again)
    glVertex3f( x - hs, y + hs, z - hs );
    glVertex3f( x - hs, y - hs, z - hs );
    glEnd();
    //*/  
    glBegin( GL_TRIANGLE_STRIP );
    // top-front edge
    glVertex3f( x - hs, y + hs, z - hs );
    glVertex3f( x + hs, y + hs, z - hs );
      
    // top-back edge
    glVertex3f( x - hs, y + hs, z + hs );
    glVertex3f( x + hs, y + hs, z + hs );
    glEnd();
    
    glBegin( GL_TRIANGLE_STRIP );
    // bottom-front edge
    glVertex3f( x - hs, y - hs, z - hs );
    glVertex3f( x + hs, y - hs, z - hs );
        
    // bottom-back edge
    glVertex3f( x - hs, y - hs, z + hs );
    glVertex3f( x + hs, y - hs, z + hs );
    glEnd();  
  
} 

static void _draw(BotViewer *viewer, BotRenderer *renderer)
{
    KinectSkeletonRenderer *self = (KinectSkeletonRenderer*) renderer->user;
    if (!self->skeleton_msg/* || !self->point_list_msg*/)
        return;

    glRotatef(90, 1, 0, 0);

    // // Draw point cloud
    // glEnable(GL_DEPTH_TEST);
    // glBegin(GL_POINTS);

    // for (int j=0; j<self->point_list_msg->num_points; j++) { 
    //     glColor3f( self->point_list_msg->points[j].r / 255, 
    //                self->point_list_msg->points[j].g / 255, 
    //                self->point_list_msg->points[j].b / 255);
    //     glVertex3f(self->point_list_msg->points[j].x, 
    //                self->point_list_msg->points[j].y, 
    //                self->point_list_msg->points[j].z);
    // }
    // glEnd();

    // // Draw point cloud debug
    // for (int j=0; j<self->point_list_msg_debug->num_points; j++) { 
    //     glColor3f( self->point_list_msg_debug->points[j].r / 255, 
    //                self->point_list_msg_debug->points[j].g / 255, 
    //                self->point_list_msg_debug->points[j].b / 255);
    //     draw_cube(self->point_list_msg_debug->points[j].x, 
    //               self->point_list_msg_debug->points[j].y, 
    //               self->point_list_msg_debug->points[j].z, 20);

    // }


    // Draw Skeleton along with point cloud
    glLineWidth(5);    
    glColor3f(.7, .1, .1);
    glBegin(GL_LINES);
    for (int i=0; i<self->skeleton_msg->num_links; i++) { 
        //if (i==0)
        //    printf("%3.2f, %3.2f, %3.2f\n", self->skeleton_msg->links[0], 
        //self->skeleton_msg->links[1], self->skeleton_msg->links[2]);
        const kinect_link_msg_t& link = self->skeleton_msg->links[i];
        glVertex3f(link.source.x, link.source.y, link.source.z);
        glVertex3f(link.dest.x, link.dest.y, link.dest.z);
    }
    glEnd();
}

static void _free(BotRenderer *renderer)
{
    KinectSkeletonRenderer *self = (KinectSkeletonRenderer*) renderer;

    // if(self->point_list_msg)
    //     kinect_point_list_t_destroy(self->point_list_msg);

    // if(self->point_list_msg_debug)
    //     kinect_point_list_t_destroy(self->point_list_msg_debug);

    if(self->skeleton_msg)
        kinect_skeleton_msg_t_destroy(self->skeleton_msg);

    kinect_calib_destroy(self->kcal);

    //free(self->disparity);
    if(self->kinect_frame)
        free(self->kinect_frame);

    free(self);
}

void 
kinect_add_skeleton_renderer_to_viewer(BotViewer* viewer, int priority, lcm_t* lcm, BotFrames * frames, const char * kinect_frame)
{
    KinectSkeletonRenderer *self = (KinectSkeletonRenderer*) calloc(1, sizeof(KinectSkeletonRenderer));

    //self->disparity = 0;

    self->frames = frames;
    if (self->frames!=NULL)
      self->kinect_frame = strdup(kinect_frame);

    //self->depth_msg = NULL;
    // self->point_list_msg = NULL;
    // self->point_list_msg_debug = NULL;
    self->skeleton_msg = NULL;

    BotRenderer *renderer = &self->renderer;

    self->kcal = kinect_calib_new();
    self->kcal->width = 640;
    self->kcal->height = 480;

    self->kcal->intrinsics_depth.fx = 576.09757860;
    self->kcal->intrinsics_depth.cx = 321.06398107;
    self->kcal->intrinsics_depth.cy = 242.97676897;

    self->kcal->intrinsics_rgb.fx = 528.49404721;
    self->kcal->intrinsics_rgb.cx = 319.50000000;
    self->kcal->intrinsics_rgb.cy = 239.50000000;
    self->kcal->intrinsics_rgb.k1 = 0;
    self->kcal->intrinsics_rgb.k2 = 0;

    self->kcal->shift_offset = 1093.4753;
    self->kcal->projector_depth_baseline = 0.07214;;

    double R[9] = { 0.999999, -0.000796, 0.001256, 0.000739, 0.998970, 0.045368, -0.001291, -0.045367, 0.998970 };
    double T[3] = { -0.015756, -0.000923, 0.002316 };

    memcpy(self->kcal->depth_to_rgb_rot, R, 9*sizeof(double));
    memcpy(self->kcal->depth_to_rgb_translation, T, 3*sizeof(double));

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

    //kinect_depth_msg_t_subscribe(self->lcm, "KINECT_DEPTH_FRAME", on_kinect_frame, self);
    //kinect_point_list_t_subscribe(self->lcm, "KINECT_POINT_CLOUD", on_kinect_point_cloud, self);
    //kinect_point_list_t_subscribe(self->lcm, "KINECT_POINT_CLOUD_DEBUG", on_kinect_point_cloud_debug, self);
    kinect_skeleton_msg_t_subscribe(self->lcm, "KINECT_SKELETON_FRAME", on_skeleton_frame, self);

    bot_viewer_add_renderer(viewer, renderer, priority);
}
