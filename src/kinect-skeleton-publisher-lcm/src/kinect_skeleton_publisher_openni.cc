#include "kinect_skeleton_publisher_openni.h"

int debug_mode=0;
XnBool g_bNeedPose   = FALSE;
XnChar g_strPose[20] = "";
    
static void usage(const char* progname)
{
  fprintf (stderr, "Usage: %s [options]\n"
                   "\n"
                   "Options:\n"
                   "  -l URL    Specify LCM URL\n"
                   "  -h        This help message\n", 
                   g_path_get_basename(progname));
  exit(1);
}

void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
	printf("New User %d", nId);

	if (g_bNeedPose)
		g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
	else
		g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}

void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
	printf("Lost user %d", nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie) {
	printf("Calibration started for user %d", nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie) {
	if (bSuccess) {
		printf("Calibration complete, start tracking user %d\n", nId);
		g_UserGenerator.GetSkeletonCap().StartTracking(nId);
	}
	else {
		printf("Calibration failed for user %d\n", nId);
		if (g_bNeedPose)
			g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
		else
			g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
	}
}

void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, XnChar const* strPose, XnUserID nId, void* pCookie) {
    printf("Pose %s detected for user %d\n", strPose, nId);
    g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
    g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}

void publishTransforms(lcm_t* lcm, const char* msg_channel) {
    XnUserID users[15];
    XnUInt16 users_count = 15;
    g_UserGenerator.GetUsers(users, users_count);

    for (int i = 0; i < users_count; ++i) {
        XnUserID user = users[i];
        if (!g_UserGenerator.GetSkeletonCap().IsTracking(user))
            continue;

        kinect_skeleton_msg_t skeleton_msg;

        const int num_links = NumLinks;
        skeleton_msg.utime = 0;
        skeleton_msg.num_links = num_links;
        skeleton_msg.user_id = user;  // double

        std::vector<kinect_link_msg_t> _links(num_links);
        for (int i = 0; i < NumLinks; ++i) {

            XnSkeletonJointPosition joint_position;
            g_UserGenerator.GetSkeletonCap().
                GetSkeletonJointPosition(user, skelLinkList(i).source, joint_position);
            double x = -joint_position.position.X / 1000.0;
            double y = joint_position.position.Y / 1000.0;
            double z = joint_position.position.Z / 1000.0;
            _links[i].joint_id = skelLinkList(i).source;
            _links[i].source.x = x, _links[i].source.y = y, _links[i].source.z = z;

            g_UserGenerator.GetSkeletonCap().
                GetSkeletonJointPosition(user, skelLinkList(i).dest, joint_position);
            x = -joint_position.position.X / 1000.0;
            y = joint_position.position.Y / 1000.0;
            z = joint_position.position.Z / 1000.0;

            _links[i].joint_id = skelLinkList(i).dest;
            _links[i].dest.x = x, _links[i].dest.y = y, _links[i].dest.z = z;

        }
        skeleton_msg.links = &_links[0];
        kinect_skeleton_msg_t_publish(lcm, msg_channel, &skeleton_msg);
    }

        return;
}


int main(int argc, char** argv)
{

    int c;
    char *lcm_url = NULL;
    // command line options - to publish on a specific url  
    while ((c = getopt (argc, argv, "hdir:jq:zl:")) >= 0) {
        switch (c) {
        case 'l':
            lcm_url = strdup(optarg);
            printf("Using LCM URL \"%s\"\n", lcm_url);
            break;
        case 'd':
            debug_mode=1;
            printf("Debug mode\n");
            break;
        case 'h':
        case '?':
            usage(argv[0]);
        }
    }

    // LCM-related
    lcm_t* lcm = lcm_create(lcm_url);
    char* msg_channel = g_strdup("KINECT_SKELETON_FRAME");

    if(!lcm) {
        printf("Unable to initialize LCM\n");
        return 1;
    }


    char config_file[2048];
    sprintf(config_file, "%s/openni_tracker.xml", getConfigPath());
    printf("Initialize from xml: %s\n", config_file);
    XnStatus nRetVal = g_Context.InitFromXmlFile(config_file);
    CHECK_RC(nRetVal, "InitFromXml");

    // Depth Generator
    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
    CHECK_RC(nRetVal, "Find depth generator");

    // User Generator
    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
    if (nRetVal != XN_STATUS_OK) {
        nRetVal = g_UserGenerator.Create(g_Context);
        CHECK_RC(nRetVal, "Find user generator");
    }

    // Skeleton Cap
    if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON)) {
        printf("Supplied user generator doesn't support skeleton\n");
        return 1;
    }

    XnCallbackHandle hUserCallbacks;
    g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);


    XnCallbackHandle hCalibrationCallbacks;
    g_UserGenerator.GetSkeletonCap().RegisterCalibrationCallbacks(UserCalibration_CalibrationStart, UserCalibration_CalibrationEnd, NULL, hCalibrationCallbacks);

    if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration()) {
        g_bNeedPose = TRUE;
        if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION)) {
            printf("Pose required, but not supported\n");
            return 1;
        }

        XnCallbackHandle hPoseCallbacks;
        g_UserGenerator.GetPoseDetectionCap().RegisterToPoseCallbacks(UserPose_PoseDetected, NULL, NULL, hPoseCallbacks);

        g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);
    }

    g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

    nRetVal = g_Context.StartGeneratingAll();
    CHECK_RC(nRetVal, "StartGenerating");
    printf("Skeleton tracker running..\nWaiting for calibration pose.. \n");

    while (true) { 
        g_Context.WaitAndUpdateAll();
        publishTransforms(lcm,msg_channel);
    }

    lcm_destroy(lcm);
  return 0;
}
