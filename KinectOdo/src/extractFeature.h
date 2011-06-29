/**
\mainpage
\htmlinclude manifest.html

\b The Pathplanner package contains a ROS service which calculates the shortest path with the help of a Dijkstra algorithm and SQLITE3 database.


\section rosapi ROS API

There is only one ROS service in this package.

List of nodes:
- \b path_server


<!-- START: copy for each node -->

<hr>

\subsection pathplanner path_server

path takes a start and end waypoint and returns the GPS-Waypoints in between based on the SQLITE3 database given to the algorithm.

\subsubsection Usage
\verbatim
$ rosservice call path [start node] [end node]
\endverbatim

\par Example

\verbatim
$ rosservice call path 1 3
\endverbatim



\subsubsection services ROS services
- \b "path": [oa_communication/path] the path service takes as input two ints and returns a nav_msgs/Odometry vector with the path.


<!-- END: copy for each node -->


//*DATABASE
/*!
 * The DATABASE class handles all the sqlite3 work within the Pathplanner package. It opens the database to a given path and
 * copies the entries into arrays. Some code is copied from an online sqlite3 tutorial (http://freshmeat.net/articles/sqlite-tutorial).
 */
#include <ros/ros.h>
#include <math.h>
#include "sensor_msgs/Image.h"
#include <sensor_msgs/Imu.h>
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "pcl_ros/io/bag_io.h"
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl/range_image/range_image.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/common/eigen.h>
#include <string>
#include <time.h>
#include <stdio.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_registration.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ctime>

#include "/opt/ros/diamondback/stacks/common_msgs/visualization_msgs/msg_gen/cpp/include/visualization_msgs/Marker.h"
#include "/opt/ros/diamondback/stacks/common_msgs/nav_msgs/msg_gen/cpp/include/nav_msgs/Odometry.h"



#include </opt/ros/diamondback/stacks/perception_pcl/pcl/include/pcl/registration/icp_nl.h>
#include "pcl/registration/icp.h"
#include "pcl/registration/icp_nl.h"

#include <nav_msgs/Path.h>

//LCMStuff
//#include "/home/benema/ros_tutorials/Kinect-Pixhawk/mavlink-ros-pkg/lcm_mavlink_ros/msg_gen/cpp/include/lcm_mavlink_ros/COMMAND.h"
#include "../../../mavlink-ros-pkg/lcm_mavlink_ros/msg_gen/cpp/include/lcm_mavlink_ros/COMMAND.h"


//sba stuff
//SBA
//#include <sba/sba.h>
//#include <sba/visualization.h>



//rgbstuff
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointXYZ Point;
#define depth_topic "/camera/depth/points"
//SBA
//#define STORE_POINTS_IN_SBA sys.addPoint(temppoint);


//typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
//typedef pcl::PointXYZRGB Point;
//#define depth_topic "/camera/rgb/points"
//#define STORE_POINTS_IN_SBA sys.addPoint(temppoint,SBAPoint.at(i).PointXYZ.rgb);

//sba
//typedef std::map<const int, sba::Proj, std::less<int>, Eigen::aligned_allocator<sba::Proj> > ProjMap;

//* Class to compute the visual Odometry with a Kinect
/**
 * Class to compute the visual Odometry with a Kinect. The position is published with a ROS publisher.
 */
class EXTRACT
{
private:
    bool slammed;
    bool notcopied;

	//SBA stuff
//	int maxx;
//	int maxy;
//	uint project_counter;
//	sba::SysSBA sys;

	//Variables for //SBA
//	frame_common::CamParams cam_params;
//	vector<Eigen::Vector4d > points;

	//Vector for correspondcense
	std::vector<int> correspondences;
	std::vector<int> correspondences_matches;
	std::vector<int> correspondences_inliers;
	std::vector<int> correspondences_source_good;
	std::vector<int> correspondences_outliers;

	std::vector<int> correspondences_source;
	std::vector<int> correspondences_target;
	std::vector<int> correspondences_target_good;
	std::vector<int> corr1,corr2;
	//Matrix for transformationrat
	Eigen::Matrix4f transformNew;
	Eigen::Matrix4f transform1;
	Eigen::Matrix4f transform2;
	Eigen::Matrix4f transform3;
	Eigen::Matrix4f imuRot;

//to fly
    ros::Subscriber imuSubscriber;
    ros::Subscriber viconSubscriber;
    ros::Subscriber commandSubscriber;
	//pointclouds
	PointCloud outputCloud;
	PointCloud outputall;
	PointCloud kinectTransformedNew;
	PointCloud kinectTransformedOld;
	PointCloud kinectTransformedTMP;
	PointCloud kinectTransformedTRY;
	//variables to compute runtime
	clock_t start,finish;
	double whole_time,odo_time,feature_time;

	//stuff for image fast image analysis:
	/// Feature Detector used for finding features in the image.
	cv::Ptr<cv::FeatureDetector> detector;
	/// Descriptor Extractor used for getting descriptors around image features.
	cv::Ptr<cv::DescriptorExtractor> extractor;


	vector<cv::DMatch> matches_popcount;
	vector<cv::DMatch> prev_matches;
	vector<vector<cv::DMatch> > matches_popcount_vector;
	//for matchihng
	cv::BruteForceMatcher<cv::Hamming> matcher_popcount;
//	cv::FlannBasedMatcher matcher_popcount;
	//variable to draw circles
	CvPoint circle;

	//variables to compute time
	time_t start_time, end_time;
	time_t start_time2,end_time2;
	time_t callback_time_start,callback_time_end;

	//which feature 1=flann 2=surf
	int feature_type;
	int callback_counter;
	int compute_counter;
	float averageNumberOfCorrespondences;
	float averageNumberOfCorrespondences_posest;
	float averageTime;
	bool called;
	bool doSLAM;
	bool called_first_time;
	bool take_vicon_z;
	int q;
	int numberOfIgnoredKeyframes;
	bool showDisplay;
	void matchFeature(cv::Mat &dtors0,cv::Mat&dtors1,	vector<cv::DMatch> &matches);
	IplImage *display_image;
	IplImage *depth_image[2];
	IplImage *cv_image[2];
	IplImage *callback_image;
	IplImage *track_reprojection_error;
	std::vector<IplImage *> track_vector;

	std::vector<int> pointsWithProjectionTo2;

	int keyframeforreprojection;

	//vector for point correspondances
    std::vector<int> ptpairs;
    pcl::PointCloud<pcl::PointXYZRGB> SURFPointCloud1;
    pcl::PointCloud<pcl::PointXYZRGB> SURFPointCloud2;

    PointCloud FeaturePointCloud[2];
    std::vector<PointCloud> Keyframes;
    int ransac_inliers;
    bool next_keyframe;

    uint counter;
//    Eigen::Matrix4f keyTrans;
    Eigen::Matrix4f previous_transformation;
	Eigen::Matrix4f /*previous_transformation_,*/ transformation_;


    std::vector<Eigen::Matrix4f> KeyframeTransformation;
    //Eigen::Matrix4f transform[2];
    pcl::PointCloud<pcl::PointXYZRGB> kinectClouds[2];
    pcl::PointCloud<pcl::PointXYZRGB> kinectCloudall;

	geometry_msgs::PoseStamped cameraPose;
	geometry_msgs::PoseStamped heliPose;
	nav_msgs::Path path;

	ros::Publisher path_pub;
	ros::Publisher bodyPoseStamped_pub;
	//SBA
//	ros::Publisher camSBA_marker_pub;
//	ros::Publisher camSBA_marker_pub_preSBA;
//	ros::Publisher pointSBA_marker_pub;
//	ros::Publisher pointSBA_marker_pub_preSBA;
	ros::Publisher cameraPose_pub;
    ros::Publisher transformedCloud;
    //for keyframes
    ros::Publisher KeyFramePoints;
    ros::Publisher KeyFrameMarkers;
    ros::Publisher SeenKeyFramePoints;
    PointCloud KeyFramePointClouds;
    nav_msgs::Odometry KeyFrameMarker;

    ros::Publisher inputCloud;
    ros::Publisher targetCloud;
    ros::Publisher inputSURFPoints;
    ros::Publisher targetSURFPoints;
    ros::Publisher sparseTransformedCloud;
    ros::Publisher cam_marker_pub;
    ros::Publisher cam_position_pub;

    ros::Publisher raw_pointcloud;
    ros::Publisher raw_picture;


	visualization_msgs::Marker camera_pos;
	nav_msgs::Odometry position;



    Eigen::Matrix4f posest_transformation;
    Eigen::Matrix4f transformGes;
    Eigen::Matrix4f transformOld;
    Eigen::Matrix4f transformRANSACOwn;


    Eigen::Quaternion<float> quat_rot;
    //to fly
    Eigen::Quaternion<float> quat_imu;
    Eigen::Quaternion<float> quat_vicon;
    Eigen::Vector3f pos_vicon;

    bool take_vicon;
    bool take_initial_vicon;
    Eigen::Matrix4f vicontransform;
    bool reset_map;
    Eigen::Quaternion<float> quat_rot_heli;
    Eigen::Quaternion<float> quat_rot_keyframe;

	Eigen::Matrix3f rot_matrix;
	Eigen::Vector3f trans_vec;
	Eigen::Matrix3f rot_matrix_heli;
	Eigen::Vector3f trans_vec_heli;
	Eigen::Vector3f trans_vec_keyframe;
	Eigen::Vector3f trans_vec_tmp;

	int countOfBadCorrespondences;
	int min_inliers;
	int next_keyframe_inlier;


	IplImage* imgadd;

    int ransac_it;
    double ransac_acc;

    std::vector<cv::KeyPoint> kpts[2];
    cv::Mat dtors[2];


	//Stuff to Extract SURF and compare:
//	CvSeq *imageKeypoints[2], *imageDescriptors[2];
//	CvSURFParams params;

	void RANSAC();
	void RANSACandTransformation();
	void transformPointcloud();

	bool showTime;
	bool doICP;
	FILE * pFile;


	bool compute_transform_success;
	//variable to define if pointcloud has RGB information
	bool colored_pointcloud;

	//Stuff for brief extractor

	CvStarDetectorParams params;

	float descriptorSizeFactor;

	CvPoint2D32f mapPoints[2048];
	CvPoint2D32f framePointsRad[2048];
	CvPoint3D32f framePoints[2048];

	// Match features, establish correspondences
	std::vector<int> briefPairs;
	std::vector<int> briefUnmatched;


	void publishEverything();
	int swap_counter;
	int number_of_swaps;

	bool transformation_at_least_once_computed;
	bool transformation_at_least_twice_computed;
	void PosEst();

	int computed;

	struct FrameData{
		Eigen::Matrix4f Transformation;
		PointCloud Points;
		cv::Mat Descriptor;
		std::vector<cv::KeyPoint> Keypoints;
		PointCloud KinectCloud;
		vector<cv::DMatch> matches_backward;
		vector<int> correspondences_backward;

	};
//SBA
//	struct SBAmap{
//		std::vector<PointCloud> Points;
//		std::vector<cv::Mat> Descriptor;
//	};
//	struct SBAmap SBAmap;

	struct mapPoint{
		Point PointXYZ;
		std::vector<std::vector<int> > cameras;
		int identifier;
	};
//SBA
//	std::vector<struct mapPoint> SBAPoint;
	int nearest_keyframe_inliers;

	struct FrameData FrameData[2];
	std::vector<struct FrameData> KeyframeDataVector;
	uint DataVectorIt;
	uint actual_keyframe;
	std::vector<int> prev_corresp;
//SBA
//	PointCloud afterSBAPointCloud;

	void swap();
	void findNearestKeyframetoLastandComputeTransformation(struct FrameData& Last);

	void createCorrespondingPointcloud(struct FrameData& Data0,PointCloud& Cloud0,struct FrameData& Data1,PointCloud& Cloud1,std::vector<int>& correspondvector,vector<cv::DMatch>& matches);
	void createCorrespondingPointcloud(struct FrameData& Data0,PointCloud& Cloud0,struct FrameData& Data1,PointCloud& Cloud1,std::vector<int>& correspondvector,vector<cv::DMatch>& matches,bool show,std::vector<cv::KeyPoint>& kpts0,std::vector<cv::KeyPoint>& kpts1);
//SBA
//	void doSBAthing(std::vector<int> correspond,struct FrameData data0,struct FrameData data1,vector<cv::DMatch> matches );
//	void doSBAwithMap();
//	void SBARANSAC(struct FrameData &Data,	vector<cv::DMatch> &matches, std::vector<int> &corresKey,std::vector<int> &corresSBA, int keyframenumber,PointCloud &sbacloud);
//	void ownSBARANSAC(struct FrameData &Data,	vector<cv::DMatch> &matches, std::vector<int> &corresKey,std::vector<int> &corresSBA,int keyframenumber,PointCloud &sbacloud);
//	void PosEst();
	void imuCallback(const sensor_msgs::Imu& imuMsg);
	void commandCallback (const lcm_mavlink_ros::COMMAND& commandMsg);
	void viconCallback (const geometry_msgs::PoseStamped& viconMsg);





public:
	EXTRACT(bool displ,float thresh, int iterations,int minimal_inliers, int keyframe_inliers,bool time, bool slam,int ignored, int near_keyframe_inliers, int swaps);
	~EXTRACT(){cvDestroyAllWindows();fclose (pFile);
	for(uint f=0; f<track_vector.size();f++)
		cvReleaseImage(&track_vector.at(f));
	/*cvReleaseMemStorage(&briefFrameStorage);cvReleaseMemStorage(&briefMapStorage);*//*cvReleaseImage(&imgadd);cvReleaseImage(&cv_image[0]);cvReleaseImage(&cv_image[1]);*//*cvReleaseImage(&callback_image[0]);cvReleaseImage(&callback_image[1]);*/};
	void display();
	void callback(const PointCloud::ConstPtr& pointCloud_ptr,const sensor_msgs::ImageConstPtr& image_ptr);
	void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr);
	PointCloud kinectCloud[2];
	PointCloud callbackCloud;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW


};
