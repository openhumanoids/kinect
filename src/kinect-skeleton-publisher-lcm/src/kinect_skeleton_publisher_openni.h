#ifndef __OPENNI_SKELETON_PUBLISHER_H__
#define __OPENNI_SKELETON_PUBLISHER_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <sys/time.h>
#include <unistd.h>
#include <inttypes.h>
#include <vector>

#include <zlib.h>
#include <glib.h>

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>

#include <lcm/lcm.h>
#include <lcmtypes/kinect_skeleton_msg_t.h>
#include <lcmtypes/kinect_link_msg_t.h>

#define WIDTH 640
#define HEIGHT 480
#define PI 3.141592
#define CHECK_RC(nRetVal, what)                                         \
    if (nRetVal != XN_STATUS_OK)                                        \
	{                                                               \
            printf("%s failed: %s\n", what, xnGetStatusString(nRetVal)); \
            return nRetVal;                                             \
	}

xn::Context        g_Context;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator  g_UserGenerator;

enum { NumLinks = 16 };
struct Link { XnSkeletonJoint source; XnSkeletonJoint dest; };

// ! List of links.
static const Link skelLinkList(int i)
{
    static const Link list[NumLinks] =  {
        {XN_SKEL_HEAD, XN_SKEL_NECK},
        {XN_SKEL_NECK, XN_SKEL_LEFT_SHOULDER},
        {XN_SKEL_LEFT_SHOULDER, XN_SKEL_LEFT_ELBOW},
        {XN_SKEL_LEFT_ELBOW, XN_SKEL_LEFT_HAND},

        {XN_SKEL_NECK, XN_SKEL_RIGHT_SHOULDER},
        {XN_SKEL_RIGHT_SHOULDER, XN_SKEL_RIGHT_ELBOW},
        {XN_SKEL_RIGHT_ELBOW, XN_SKEL_RIGHT_HAND},

        {XN_SKEL_LEFT_SHOULDER, XN_SKEL_TORSO},
        {XN_SKEL_RIGHT_SHOULDER, XN_SKEL_TORSO},

        {XN_SKEL_TORSO, XN_SKEL_LEFT_HIP},
        {XN_SKEL_LEFT_HIP, XN_SKEL_LEFT_KNEE},
        {XN_SKEL_LEFT_KNEE, XN_SKEL_LEFT_FOOT},

        {XN_SKEL_TORSO, XN_SKEL_RIGHT_HIP},
        {XN_SKEL_RIGHT_HIP, XN_SKEL_RIGHT_KNEE},
        {XN_SKEL_RIGHT_KNEE, XN_SKEL_RIGHT_FOOT},

        {XN_SKEL_LEFT_HIP, XN_SKEL_RIGHT_HIP},
    };
    return list[i];
}

#endif //  __OPENNI_SKELETON_PUBLISHER_H__
