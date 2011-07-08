#include "LOC.h"

bool verbose;
double ransac_threshold=.2;
int ransac_it=200;
int type=0;
int min_inlier=70;
int min_keyframe=130;
std::string filepath="mapdata.bin";

static GOptionEntry entries[] =
{
		{ "verbose", 'v', 0, G_OPTION_ARG_NONE, &verbose, "Be verbose", NULL },
		{ "type", 't', 0, G_OPTION_ARG_INT, &verbose, "Type of odometry: 0 -> use map 1 -> use previous image", NULL },
		{ "ransac_threshold", 't', 0, G_OPTION_ARG_DOUBLE, &ransac_threshold, "inlier threshold for RANSAC", ".2" },
		{ "ransac_it", 'i', 0, G_OPTION_ARG_INT, &ransac_it, "Maximum iterations for RANSAC", "200" },
		{ "min_inlier", 'm', 0, G_OPTION_ARG_INT, &min_inlier, "Minimal inlier to compute transformation (only for odometry to last frame)", "70" },
		{ "min_keyframe", 'k', 0, G_OPTION_ARG_INT, &min_keyframe, "Minimal inlier to keep the same Keyframe (only for odometry to last frame)", "130" },
		{ "filepath", 'f', 0, G_OPTION_ARG_STRING, &filepath, "path of file in which map is stored", "mapdata.bin" },
		{ NULL, 0, 0, G_OPTION_ARG_NONE, NULL, NULL, 0 }

};

int main(int argc, char **argv)
{

	GError *error = NULL;
	GOptionContext *context;

	context = g_option_context_new("START LOCALIZATION");
	g_option_context_add_main_entries(context, entries, NULL);
	if (!g_option_context_parse(context, &argc, &argv, &error))
	{
		g_print("Option parsing failed: %s\n", error->message);
		exit(1);
	}

	ros::init(argc, argv, "KinectOnlineLocalization");
		std::cout<<"waiting for kinect data"<<std::endl;
		std::cout<<"ransac_threshold:"<<ransac_threshold<<"ransac_it:"<<ransac_it<<std::endl
				<<"min_inliers:"<<min_inlier<<"min_keyframe:"<<min_keyframe<<"verbose:"<<verbose<<std::endl
				<<"type:"<<type<<"filepath:"<<filepath.c_str()<<std::endl;
	LOC::LOC loc(ransac_threshold, ransac_it,min_inlier,min_keyframe,verbose, type,filepath);

	return 0;
}
