#include <glib.h>
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
//maybe
//#include <math.h>
//
#include "sensor_msgs/Image.h"
#include <sensor_msgs/Imu.h>
//maybe
//#include "image_transport/image_transport.h"
//
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
//maybe
//#include <string>
//#include <time.h>
//#include <stdio.h>
//
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_registration.h>
//maybe
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <ctime>
//#include "/opt/ros/diamondback/stacks/common_msgs/visualization_msgs/msg_gen/cpp/include/visualization_msgs/Marker.h"
//#include "/opt/ros/diamondback/stacks/common_msgs/nav_msgs/msg_gen/cpp/include/nav_msgs/Odometry.h"
//#include </opt/ros/diamondback/stacks/perception_pcl/pcl/include/pcl/registration/icp_nl.h>
//#include "pcl/registration/icp.h"
//#include "pcl/registration/icp_nl.h"
//#include <nav_msgs/Path.h>
//
//LCMStuff
//#include "/home/benema/ros_tutorials/Kinect-Pixhawk/mavlink-ros-pkg/lcm_mavlink_ros/msg_gen/cpp/include/lcm_mavlink_ros/COMMAND.h"



//to get commands from MAVLINK: 200, Stop mapping: 201
#include "../../../mavlink-ros-pkg/lcm_mavlink_ros/msg_gen/cpp/include/lcm_mavlink_ros/COMMAND.h"

//TORO
//#include "../../toro/trunk/treeoptimizer2.hh"
#include "../../toro/trunk/treeoptimizer3.hh"
//#include "../../toro/trunk/posegraph.hh"
//#include "../../toro/trunk/posegraph2.hh"
#include "../../toro/trunk/posegraph3.hh"
//#include "../../toro/trunk/transformation2.hh"
//#include "../../toro/trunk/transformation3.hh"
//#include "../../toro/trunk/dmatrix.hh"


//sba stuff
//SBA
//#include <sba/sba.h>
//#include <sba/visualization.h>





//Typedefs for PCL
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointXYZ Point;

class MAP
{
private:
	///******CLASS VARIABLES

	//***ROS variables
	ros::Subscriber imuSubscriber;
	ros::Subscriber viconSubscriber;
	ros::Subscriber commandSubscriber;
    ros::Publisher KeyFramePoints;


	//Parameters from the programm start
	bool showDisplay;
	float ransac_acc;
	int ransac_it;
	int min_inliers;
	int min_keyframe_inlier;
	bool showtext;
	int min_keyframe_redection_inliers;
	string path;
	int init;

	//stuff for image fast image analysis:
	/// Feature Detector used for finding features in the image.
	cv::Ptr<cv::FeatureDetector> detector;
	/// Descriptor Extractor used for getting descriptors around image features.
	cv::Ptr<cv::DescriptorExtractor> extractor;
	cv::BruteForceMatcher<cv::Hamming> matcher_popcount;



	//other variables
	bool called;
	int counter;
	bool called_first_time;
	IplImage *cv_image[2];
	IplImage *callback_image;
	IplImage* imgadd;
	CvPoint circle;
	PointCloud kinectCloud[2];
	PointCloud callbackCloud;
    PointCloud FeaturePointCloud[2];
    std::vector<cv::KeyPoint> kpts[2];
    cv::Mat dtors[2];
	vector<cv::DMatch> matches_popcount;
	std::vector<int> correspondences;
	std::vector<int> correspondences_source_good;
	std::vector<int> correspondences_inliers;
	std::vector<int> correspondences_target_good;
	bool compute_transform_success;
	bool transformation_at_least_once_computed;
    Eigen::Matrix4f previous_transformation;
	Eigen::Matrix4f transformation_;
	Eigen::Matrix4f transformOld;
	Eigen::Matrix4f vicontransform;
	Eigen::Matrix4f imuRot;
	Eigen::Matrix4f systemRot;
	Eigen::Quaternion<float> quat_imu;
//	Eigen::Quaternion<float> quat_rot;
	Eigen::Quaternion<float> quat_vicon;
	Eigen::Vector3f pos_vicon;
	bool vicon_computed;
	bool imu_computed;
    bool take_vicon;
    bool stop_mapping;
    int actual_keyframe;
    bool next_keyframe;
    int ransac_inliers;
    bool transformation_at_least_twice_computed;




    //*** structs
	struct FrameData{
		Eigen::Matrix4f Transformation;
		PointCloud Points;
		cv::Mat Descriptor;
		std::vector<cv::KeyPoint> Keypoints;
		PointCloud KinectCloud;
		vector<int> edges_to;
		vector<Eigen::Matrix4f> Transformation_to;
		vector<cv::DMatch> matches_backward;
		vector<int> correspondences_backward;

	};
	struct FrameData FrameData[2];
	std::vector<struct FrameData> KeyframeDataVector;
	std::vector<struct FrameData> finalMAP;

	struct MapData{
		PointCloud Points;
		cv::Mat Descriptor;
	};

	struct MapData refinedMap;

	struct mapPoint{
		Point PointXYZ;
		std::vector<std::vector<int> > cameras;
		int identifier;
	};

	std::vector<struct mapPoint> SBAPoint;



	///******CLASS FUNCTIONS
	void matchFeature(cv::Mat &dtors0,cv::Mat&dtors1,	vector<cv::DMatch> &matches);
	void RANSACandTransformation();
	void createCorrespondingPointcloud(struct FrameData& Data0,PointCloud& Cloud0,struct FrameData& Data1,PointCloud& Cloud1,std::vector<int>& correspondvector,vector<cv::DMatch>& matches,bool show,std::vector<cv::KeyPoint>& kpts0,std::vector<cv::KeyPoint>& kpts1);
	void RANSAC();
	void swap();
	void refineMapWithTORO(std::vector<struct FrameData>* map);
	void findConnectionsBetweenKeyframes(int number,struct FrameData& from, std::vector<struct FrameData>& to);





	//***ROS callback functions
	void viconCallback (const geometry_msgs::PoseStamped& viconMsg);
	void commandCallback (const lcm_mavlink_ros::COMMAND& commandMsg);
	void imuCallback (const sensor_msgs::Imu& imuMsg);
	void callback(const PointCloud::ConstPtr& pointCloud_ptr,const sensor_msgs::ImageConstPtr& image_ptr);


public:
	//Constructor for the class
	MAP(float thresh, int iterations,int minimal_inliers, int keyframe_inliers,bool verbose, int near_keyframe_inliers,string filepath,int initialization);
	//Destructor for the class
	~MAP(){};
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW




};
