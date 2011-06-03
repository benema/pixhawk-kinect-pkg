#include "extractFeature.h"


int main(int argc, char **argv)
{
	bool display;
//	int type;
	float ransac_threshold;
	int ransac_it;
//	float descFact;
//	int responseThresh;
//	int lineProj;
//	int lineBin;
//	int nonmaxSupp;
//	int maxSize;
	int min_inlier;
	int min_keyframe;
	bool time;
	bool slam;
	int ignored;
	int nearest_inliers;
	int swaps;
	ros::init(argc, argv, "mainSLAM");
	if (argc>10)
	{
		display=atoi(argv[1]);
//		type=atoi(argv[2]);
		ransac_threshold=atof(argv[2]);
		ransac_it=atoi(argv[3]);
//		descFact=atof(argv[5]);
//		responseThresh=atoi(argv[6]);
//		lineProj=atoi(argv[7]);
//		lineBin=atoi(argv[8]);
//		nonmaxSupp=atoi(argv[9]);
//		maxSize=atoi(argv[10]);
		min_inlier=atoi(argv[4]);
		min_keyframe=atoi(argv[5]);
		time=atoi(argv[6]);
		slam=atoi(argv[7]);
		ignored=atoi(argv[8]);
		nearest_inliers=atoi(argv[9]);
		swaps=atoi(argv[10]);


		std::cout<<"waiting for kinect data"<<std::endl;

	}
	else
	{
		std::cout<<"use like this: mainSLAM 1 (or 0 to not display).15 (RANSAC threshold) 200 (number of ransac iterations)"
				" 70 (min_inliers for transformation) 130 (min ilniers for keyframe) 1 (show time) 0 (do slam) 5 (number of ingroedkeyframes for redetection)"
				"100 (number of in inliers to compute redeteciotn transformation) 10 (number of keyframes for slam)"<<std::endl;
				return 0;

	}
	EXTRACT::EXTRACT ext(display,ransac_threshold, ransac_it,min_inlier,min_keyframe,time, slam,ignored, nearest_inliers,swaps);
	/*ros::NodeHandle n;

	ros::AsyncSpinner s(2);
	s.start();
	ros::Rate r(15);
	while(ros::ok())
	{
		ext.callCallback();
		r.sleep();
	};*/

	return 0;
}
