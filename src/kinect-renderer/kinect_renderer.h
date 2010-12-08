#ifndef __kinect_bot_renderer_h__
#define ___bot_renderer_h__

/**
 * @defgroup KinectBotRenderer KinectBotRenderer renderer
 * @brief BotVis Viewer renderer plugin
 * @include kinect-renderer/kinect_renderer.h
 *
 * TODO
 *
 * Linking: `pkg-config --libs kinect-renderer`
 * @{
 */

#include <lcm/lcm.h>

#include <bot_vis/bot_vis.h>

#ifdef __cplusplus
extern "C" {
#endif

void kinect_add_renderer_to_viewer(BotViewer* viewer, lcm_t* lcm, int priority);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
