#include "MAP.h"

bool verbose;
double ransac_threshold=.2;
int ransac_it=200;
int min_inlier=70;
int min_keyframe=130;
int nearest_inliers=100;
int initialization=0;
std::string filepath="mapdata.bin";

// Handling Program options
static GOptionEntry entries[] =
{
		{ "verbose", 'v', 0, G_OPTION_ARG_NONE, &verbose, "Be verbose", NULL },
		{ "initializiation", 'z', 0, G_OPTION_ARG_INT, &initialization, "0 start with Identity, 1 start with IMU, 2 start with Vicon", NULL },
		{ "ransac_threshold", 't', 0, G_OPTION_ARG_DOUBLE, &ransac_threshold, "inlier threshold for RANSAC", ".2" },
		{ "ransac_it", 'i', 0, G_OPTION_ARG_INT, &ransac_it, "Maximum iterations for RANSAC", "200" },
		{ "min_inlier", 'm', 0, G_OPTION_ARG_INT, &min_inlier, "Minimal inlier to compute transformation", "70" },
		{ "min_keyframe", 'k', 0, G_OPTION_ARG_INT, &min_keyframe, "Minimal inlier to keep the same Keyframe", "130" },
		{ "nearest_inliers", 'n', 0, G_OPTION_ARG_INT, &nearest_inliers, "Number of nearest inliers necessary to compute transformation to another keyframe", "100" },
		{ "filepath", 'f', 0, G_OPTION_ARG_STRING, &filepath, "path of file in which map is stored", "mapdata.bin" },
		{ NULL, 0, 0, G_OPTION_ARG_NONE, NULL, NULL, 0 }

};

int main(int argc, char **argv)
{

	GError *error = NULL;
	GOptionContext *context;

	context = g_option_context_new("START KINECTODO");
	g_option_context_add_main_entries(context, entries, NULL);
	if (!g_option_context_parse(context, &argc, &argv, &error))
	{
		g_print("Option parsing failed: %s\n", error->message);
		exit(1);
	}

	ros::init(argc, argv, "KinectOfflineSLAM");
		std::cout<<"waiting for kinect data"<<std::endl;
		std::cout<<"ransac_threshold:"<<ransac_threshold<<"ransac_it:"<<ransac_it<<"initializiation:"<<initialization<<std::endl
				<<"min_inliers:"<<min_inlier<<"min_keyframe:"<<min_keyframe<<"verbose:"<<verbose<<std::endl
				<<"nearest_inliers:"<<nearest_inliers<<"filepath:"<<filepath.c_str()<<std::endl;
	MAP::MAP map(ransac_threshold, ransac_it,min_inlier,min_keyframe,verbose, nearest_inliers,filepath,initialization);

	return 0;
}
