#include "extractFeature.h"

static CvScalar colors[] =
{
		{{0,0,255}},
		{{0,128,255}},
		{{0,255,255}},
		{{0,255,0}},
		{{255,128,0}},
		{{255,255,0}},
		{{255,0,0}},
		{{255,0,255}},
		{{255,255,255}}
};



//code partially copied from http://www.cs.unc.edu/~blloyd/comp290-089/fmatrix/
void EXTRACT::RANSAC()
{

	std::cout<<"inransac"<<std::endl;


	Eigen::Matrix4f frametrans=Eigen::Matrix4f::Identity();
	correspondences_source_good.clear();
	correspondences_target_good.clear();
	correspondences_outliers.clear();
	correspondences_source=correspondences;
	correspondences_target=correspondences;
	Eigen::Vector3f translation;


	/***Defining all the RANSAC STUFF***/

	//	if(colored_pointcloud==true)
	//	{
	//		typedef pcl::SampleConsensusModelRegistration<pcl::PointXYZRGB>::Ptr SampleConsensusModelRegistrationPtrRGB;
	//	SampleConsensusModelRegistrationPtrRGB modelRGB;
	//	modelRGB.reset (new pcl::SampleConsensusModelRegistrationRGB<pcl::PointXYZRGB> (FeaturePointCloud[1].makeShared (), correspondences));
	//	modelRGB->setInputTarget (FeaturePointCloud[0].makeShared(), correspondences);
	//	pcl::RandomSampleConsensus<pcl::PointXYZRGB> sac (modelRGB, ransac_acc);
	//	sac.setMaxIterations (ransac_it);
	//
	//	}

	typedef pcl::SampleConsensusModelRegistration<Point>::Ptr SampleConsensusModelRegistrationPtr;

	SampleConsensusModelRegistrationPtr model;


	model.reset (new pcl::SampleConsensusModelRegistration<Point> (FeaturePointCloud[1].makeShared (), correspondences));
	// Pass the target_indices


	std::cout<<"befor featurecloud0"<<std::endl;
	model->setInputTarget (FeaturePointCloud[0].makeShared(), correspondences);
	// Create a RANSAC model
	std::cout<<"after featurecloud0"<<std::endl;

	pcl::RandomSampleConsensus<Point> sac (model, ransac_acc);
	sac.setMaxIterations (ransac_it);


	// Compute the set of inliers

	if (!sac.computeModel (5))
	{
		correspondences_source_good = correspondences_source;
		//		correspondences_target_good = correspondences_target;
	}
	else
	{
		std::vector<int> inliers;
		// Get the inliers
		sac.getInliers (inliers);
		//		correspondences_source_good.resize (inliers.size ());
		//		correspondences_target_good.resize (inliers.size ());
		//		// Copy just the inliers
		//		for (size_t i = 0; i < inliers.size (); ++i)
		//		{
		//			correspondences_source_good[i] = correspondences_source[inliers[i]];
		//			correspondences_target_good[i] = correspondences_target[inliers[i]];
		//		}
		//	}
		int c=0;
		for (size_t i = 0; i < correspondences.size (); ++i)
		{
			if(i==inliers[c])
			{
				correspondences_source_good.push_back(correspondences[inliers[c]]);
				c++;
			}
			else
				correspondences_outliers.push_back(correspondences[i]);
		}


		//	std::cout<<"size of keyframeoriginal"<<corr_Keyframe_inliers.size()<<"size of keyframenew"<<corr_Keyframe_in.size()<<std::endl;
		//	for(uint i=0;i<corr_Keyframe_inliers.size();i++)
		//	{
		//		std::cout<<"keyframeor"<<corr_Keyframe_inliers.at(i)<<"keyframenew"<<corr_Keyframe_in.at(i)<<std::endl;
		//	}
		//	for(uint i=0;i<corr_Keyframe_outliers.size();i++)
		//		std::cout<<"outliers"<<corr_Keyframe_outliers.at(i)<<std::endl;

	}

	if(showDisplay)
	{
		ROS_INFO("number of correspondences  after ransac %d",correspondences_source_good.size());
		averageNumberOfCorrespondences+=correspondences_source_good.size();
		ROS_INFO("average correspondences=%f",averageNumberOfCorrespondences/float(compute_counter));
	}
	ransac_inliers=correspondences_source_good.size();

	//	std::cout<<"minransac keyfram"<<next_keyframe_inlier<<std::endl;
	//	std::cout<<"next keyframe:"<<next_keyframe<<std::endl;
	//Check whether we need next keyframe
	if(ransac_inliers<next_keyframe_inlier)
		next_keyframe=true;
	else
		next_keyframe=false;

	// Check whether we have enough correspondences

	if (ransac_inliers < min_inliers)
	{
		ROS_ERROR ("[pcl::::computeTransformation] Not enough correspondences found. Relax your threshold parameters.");
		compute_transform_success=false;
		countOfBadCorrespondences++;
	}
	else
		compute_transform_success=true;




	//Berechnung der Transformation
	if(compute_transform_success)
	{
		if(transformation_at_least_once_computed)
			transformation_at_least_twice_computed=true;
		transformation_at_least_once_computed=true;

		previous_transformation=transformation_;

		pcl::estimateRigidTransformationSVD(FeaturePointCloud[1],correspondences_source_good,FeaturePointCloud[0],correspondences_source_good,transformation_);

		std::cout<<"transformation_"<<std::endl<<transformation_<<std::endl;
		std::cout<<"KeyframeDataVector.at(actual_keyframe).Transformation"<<std::endl<<KeyframeDataVector.at(actual_keyframe).Transformation<<std::endl;
		std::cout<<"actual keyframe"<<actual_keyframe<<std::endl;
		//		if(transformation_at_least_twice_computed)
		//		{
		//			frametrans=keyTrans*transformation_*previous_transformation.inverse()*transformation_;
		//			translation=frametrans.block<3,1>(0,3);
		//			betrag=sqrt(translation[0]*translation[0]+translation[1]*translation[1]+translation[2]*translation[2]);
		//			//			if(betrag>1)
		//			//			{
		//			//				transformation_=Eigen::Matrix4f::Identity();
		//			//				ROS_ERROR ("TRANSLATION to large");
		//			//
		//			//			}
		//		}
		transformOld=KeyframeDataVector.at(actual_keyframe).Transformation*transformation_;



		//				Eigen::Matrix4f Rotz=Eigen::Matrix4f::Identity();
		//
		//				Rotz.col(0)[0]=cos(M_PI/2);
		//				Rotz.col(0)[1]=-sin(M_PI/2);
		//				Rotz.col(1)[0]=sin(M_PI/2);
		//				Rotz.col(1)[1]=cos(M_PI/2);
		//
		//			//	Eigen::Matrix4f Roty=Eigen::Matrix4f::Identity();
		//			//
		//			//	Roty.col(0)[0]=cos(-M_PI/2);
		//			//	Roty.col(0)[2]=sin(-M_PI/2);
		//			//	Roty.col(2)[0]=-sin(-M_PI/2);
		//			//	Roty.col(2)[2]=cos(-M_PI/2);
		//
		//				Eigen::Matrix4f Rotx=Eigen::Matrix4f::Identity();
		//				Rotx.col(1)[1]=cos(M_PI/2);
		//				Rotx.col(1)[2]=-sin(M_PI/2);
		//				Rotx.col(2)[1]=sin(M_PI/2);
		//				Rotx.col(2)[2]=cos(M_PI/2);
		//			Eigen::Matrix3f matrix(quat_vicon);
		//
		//			Eigen::Matrix4f vicontransform=Eigen::Matrix4f::Identity();
		//
		//			vicontransform.block<3,1>(0,3)=pos_vicon;
		//			vicontransform.block<3,3>(0,0)=matrix;
		//			transformOld=Rotz*Rotx*vicontransform;
		//
		//			std::cout<<"vicontransform:"<<std::endl<<vicontransform<<std::endl;
		//			}



	}
	else
	{

		if(transformation_at_least_twice_computed)
		{
			frametrans=KeyframeDataVector.at(actual_keyframe).Transformation*transformation_*previous_transformation.inverse()*transformation_;
			translation=frametrans.block<3,1>(0,3);
			//					betrag=sqrt(translation[0]*translation[0]+translation[1]*translation[1]+translation[2]*translation[2]);
			//			if(betrag>1)
			//			{
			//				frametrans=Eigen::Matrix4f::Identity();
			//				ROS_ERROR ("TRANSLATION to large");
			//
			//			}
			std::cout<<"frametrans"<<frametrans<<std::endl;
			transformOld=frametrans;
		}
		else
			transformOld=Eigen::Matrix4f::Identity();


	}
	if(take_vicon)
	{
		next_keyframe=true;

		//		vicontransform.block<3,3>(0,0)=matrix;
		transformOld=vicontransform;

		std::cout<<"vicontransform:"<<std::endl<<vicontransform<<std::endl;

		take_vicon=false;
	}
	//	if(next_keyframe)
	//	{
	//		keyTrans=transformOld;
	//
	//	}

	std::cout<<"transformold"<<std::endl<<transformOld<<std::endl;
	correspondences_inliers=correspondences_source_good;

}

/* Funktion um die Pointcloud zu transformieren. nur gebraucht für SLAM */

void EXTRACT::transformPointcloud()
{
	pcl::transformPointCloud(kinectCloud[1],kinectTransformedOld,transformOld);
	kinectTransformedOld.header.stamp=ros::Time::now();

}

/* Function to publish all the necessary ROS variables */
void EXTRACT::publishEverything()
{
	// variable for the position of the camera (used only for visualization)
	cameraPose.header.frame_id="/pgraph";
	cameraPose.pose.position.x=trans_vec[0];
	cameraPose.pose.position.y=trans_vec[1];
	cameraPose.pose.position.z=trans_vec[2];

	cameraPose.pose.orientation.w=quat_rot.w();
	cameraPose.pose.orientation.x=quat_rot.x();
	cameraPose.pose.orientation.y=quat_rot.y();
	cameraPose.pose.orientation.z=quat_rot.z();

	cameraPose_pub.publish(cameraPose);

	//    Eigen::Quaternion<float> quat_rot_transform;
	//    quat_rot_transform


	heliPose.header.frame_id="/pgraph";
	heliPose.header.stamp=ros::Time::now();
	heliPose.pose.position.x=trans_vec[2];
	heliPose.pose.position.y=trans_vec[0];
	if(take_vicon_z)
	{
		heliPose.pose.position.z=pos_vicon[2];
		std::cout<<"taking vicon z"<<std::endl;
	}
	else
		heliPose.pose.position.z=trans_vec[1];

	Eigen::Quaternion<float> quat_tmp;
	quat_tmp.w()=quat_rot.w();
	quat_tmp.x()=quat_rot.z();
	quat_tmp.y()=quat_rot.x();
	quat_tmp.z()=quat_rot.y();

	//	quat_tmp=quat_imu*quat_tmp;

	heliPose.pose.orientation.w=quat_tmp.w();
	heliPose.pose.orientation.x=quat_tmp.x();
	heliPose.pose.orientation.y=quat_tmp.y();
	heliPose.pose.orientation.z=quat_tmp.z();




	//imuMutex_.lock();
	//		btQuaternion q(quat_rot_heli.x(), quat_rot_heli.y(), quat_rot_heli.z(), quat_rot_heli.w());
	//		btMatrix3x3 m(q);
	//		std::cout<<"m:"<<m.getRow(0)[0]<<" "<<m.getRow(0)[1]<<" "<<m.getRow(0)[2]<<std::endl
	//				<<" "<<m.getRow(1)[0]<<" "<<m.getRow(1)[1]<<" "<<m.getRow(1)[2]<<std::endl
	//				<<" "<<m.getRow(2)[0]<<" "<<m.getRow(2)[1]<<" "<<m.getRow(2)[2]<<std::endl;
	//		double Roll, Pitch, Yaw;
	//		m.getRPY(Roll, Pitch, Yaw);
	//
	//		std::cout<<"rollend:"<<Roll<<"pitchend"<<Pitch<<"Yawend"<<Yaw<<std::endl;

	bodyPoseStamped_pub.publish(heliPose);




	//variable for the path of the camera (used only for visualization)
	path.header.frame_id="/pgraph";
	path.poses.push_back(cameraPose);

	path_pub.publish(path);


	if(doSLAM)
	{
		//SBA
		//		std::cout<<"copy points"<<SBAmap.Points.size()<<std::endl;
		//		PointCloud tmp;
		//		tmp.header.frame_id="/pgraph";
		//	for(uint i=0;i<SBAPoint.size();i++)
		//		tmp.points.push_back(SBAPoint.at(i).PointXYZ);
		//	KeyFramePoints.publish(tmp);

	}
	else
		KeyFramePoints.publish(KeyFramePointClouds);

	//	for(uint i=0;i<KeyframeDataVector.size();i++)
	//		tmp+=KeyframeDataVector.at(i).Points;

	//	if(next_keyframe)
	//	{
	//		//Publish Camera poses for SBA
	//				camera_pos.pose.orientation.x=quat_rot.x()/quat_rot.w();
	//				camera_pos.pose.orientation.y=quat_rot.y()/quat_rot.w();
	//				camera_pos.pose.orientation.z=quat_rot.z()/quat_rot.w();
	//				camera_pos.header.frame_id="/pgraph";
	//				ros::Time tstamp=ros::Time::now();
	//				camera_pos.header.stamp=tstamp;
	//				camera_pos.type=visualization_msgs::Marker::ARROW;
	//				camera_pos.scale.x=1;
	//				camera_pos.scale.y=1;
	//				camera_pos.scale.z=1;
	//				camera_pos.pose.position.x=trans_vec[0];
	//				camera_pos.pose.position.y=trans_vec[1];
	//				camera_pos.pose.position.z=trans_vec[2];
	//				camera_pos.ns = "my_namespace";
	//				camera_pos.id = 0;
	//				camera_pos.color.a = 1.0f;
	//				camera_pos.color.r = 0.0f;
	//				camera_pos.color.g = 1.0f;
	//				camera_pos.color.b = 0.0f;
	//				KeyFrameMarkers.publish(camera_pos);
	//	}




}

void EXTRACT::RANSACandTransformation()
{
	if(actual_keyframe==(KeyframeDataVector.size()-1))
	{
		createCorrespondingPointcloud(FrameData[0],FeaturePointCloud[0],FrameData[1],FeaturePointCloud[1],correspondences,matches_popcount,showDisplay,kpts[0],kpts[1]);
	}
	else
		createCorrespondingPointcloud(KeyframeDataVector.at(actual_keyframe),FeaturePointCloud[0],FrameData[1],FeaturePointCloud[1],correspondences,matches_popcount);
	RANSAC();

	//	PosEst();

	time_t start_nearest=clock();
	//	if(actual_keyframe==(KeyframeDataVector.size()-1))
	findNearestKeyframetoLastandComputeTransformation(FrameData[counter]);
	//	else
	//		findNearestKeyframetoLastandComputeTransformation(FrameData[1]);

	time_t end_nearest=clock();
	std::cout<<"time for find nearest and recompute"<<(double(end_nearest)-double(start_nearest))/double(CLOCKS_PER_SEC);

	rot_matrix=transformOld.block<3,3>(0,0);
	trans_vec=transformOld.block<3,1>(0,3);
	Eigen::Quaternion<float> quat_test(rot_matrix);
	quat_rot=quat_test;

	Eigen::Matrix4f Rotz=Eigen::Matrix4f::Identity();

	Rotz.col(0)[0]=cos(-M_PI/2);
	Rotz.col(0)[1]=-sin(-M_PI/2);
	Rotz.col(1)[0]=sin(-M_PI/2);
	Rotz.col(1)[1]=cos(-M_PI/2);

	//	Eigen::Matrix4f Roty=Eigen::Matrix4f::Identity();
	//
	//	Roty.col(0)[0]=cos(-M_PI/2);
	//	Roty.col(0)[2]=sin(-M_PI/2);
	//	Roty.col(2)[0]=-sin(-M_PI/2);
	//	Roty.col(2)[2]=cos(-M_PI/2);

	Eigen::Matrix4f Rotx=Eigen::Matrix4f::Identity();
	Rotx.col(1)[1]=cos(-M_PI/2);
	Rotx.col(1)[2]=-sin(-M_PI/2);
	Rotx.col(2)[1]=sin(-M_PI/2);
	Rotx.col(2)[2]=cos(-M_PI/2);

	std::cout<<"transformOld"<<std::endl<<transformOld<<std::endl;

	Eigen::Matrix4f tmpTrans=Rotx*Rotz*transformOld;
	//	std::cout<<"tmpTrans"<<std::endl<<tmpTrans<<std::endl;
	//	std::cout<<"rotx"<<std::endl<<Rotx<<std::endl;
	//	std::cout<<"rotz"<<std::endl<<Rotz<<std::endl;

	rot_matrix_heli=tmpTrans.block<3,3>(0,0);
	trans_vec_heli=tmpTrans.block<3,1>(0,3);
	Eigen::Quaternion<float> quat_test_heli(rot_matrix_heli);
	quat_rot_heli=quat_test_heli;

	FrameData[counter].Transformation=transformOld;

	//	doSBAthing(correspondences_source_good,FrameData[0],FrameData[1],matches_popcount);

	if(showDisplay)
	{

		{
			//		if(showDisplay)
			//		{
			//			for(uint k=0;k<2;k++)
			//			{
			//				for(uint i=0;i<kpts[k].size();i++)
			//				{
			//					circle.x=kpts[k][i].pt.x;
			//					circle.y=kpts[k][i].pt.y;
			//					cvCircle( cv_image[k], circle, 1, colors[5], 1, 8, 0 );
			//				}
			//			}
			//		}
			//						std::cout<<"in showdisplay corr target_goodsize: "<<correspondences_target_good.size()<<std::endl;
			if(actual_keyframe==(KeyframeDataVector.size()-1))
			{
				ROS_WARN("in cvline target_good");

				for(uint i=0;i<correspondences_source_good.size();i++)
				{
					cvLine( imgadd, cvPoint(kpts[0][matches_popcount[correspondences_source_good[i]].queryIdx].pt.x, kpts[0][matches_popcount[correspondences_source_good[i]].queryIdx].pt.y),
							cvPoint(kpts[1][matches_popcount[correspondences_source_good[i]].trainIdx].pt.x, 480+kpts[1][matches_popcount[correspondences_source_good[i]].trainIdx].pt.y), colors[2] );

					//				cvLine( imgadd, cvPoint(kpts[0][matches_popcount[correspondences_matches[correspondences_target_good[i]]].queryIdx].pt.x, kpts[0][matches_popcount[correspondences_matches[correspondences_target_good[i]]].queryIdx].pt.y),
					//						cvPoint(kpts[1][matches_popcount[correspondences_matches[correspondences_source_good[i]]].trainIdx].pt.x, 480+kpts[1][matches_popcount[correspondences_matches[correspondences_source_good[i]]].trainIdx].pt.y), colors[2] );
				}
			}
		}
	}


	//	if(showDisplay)
	//	{
	std::cout<<"correspondences_inliers.size() RANSACINLIERS"<<correspondences_inliers.size()<<std::endl;
	//		for(uint i=0;i<correspondences_inliers.size();i++)
	//		{
	//			//				push_back_point.x=kinectCloud[0].at(kpts[0][matches_popcount[correspondences_matches[correspondences_inliers[i]]].queryIdx].pt.x,kpts[0][matches_popcount[correspondences_matches[correspondences_inliers[i]]].queryIdx].pt.y).x;
	//			//				push_back_point.y=kinectCloud[0].at(kpts[0][matches_popcount[correspondences_matches[correspondences_inliers[i]]].queryIdx].pt.x,kpts[0][matches_popcount[correspondences_matches[correspondences_inliers[i]]].queryIdx].pt.y).y;
	//			//				push_back_point.z=kinectCloud[0].at(kpts[0][matches_popcount[correspondences_matches[correspondences_inliers[i]]].queryIdx].pt.x,kpts[0][matches_popcount[correspondences_matches[correspondences_inliers[i]]].queryIdx].pt.y).z;
	//			//				std::cout<<"inlier cloud point x:"<<push_back_point.x<<"\t\t"<<"inlier point y:"<<push_back_point.y<<"\t\t"<<"inlier point z:"<<push_back_point.z<<std::endl;
	//			//				push_back_point.x=FeaturePointCloud[0].points[correspondences_inliers[i]].x;
	//			//				push_back_point.y=FeaturePointCloud[0].points[correspondences_inliers[i]].y;
	//			//				push_back_point.z=FeaturePointCloud[0].points[correspondences_inliers[i]].z;
	//			//				std::cout<<"inlier point x:"<<push_back_point.x<<"\t\t"<<"inlier point y:"<<push_back_point.y<<"\t\t"<<"inlier point z:"<<push_back_point.z<<std::endl;
	//			//				push_back_point.x=FeaturePointCloud[1].points[correspondences_inliers[i]].x;
	//			//				push_back_point.y=FeaturePointCloud[1].points[correspondences_inliers[i]].y;
	//			//				push_back_point.z=FeaturePointCloud[1].points[correspondences_inliers[i]].z;
	//			//				std::cout<<"corres point x:"<<push_back_point.x<<"\t\t"<<"inlier point y:"<<push_back_point.y<<"\t\t"<<"inlier point z:"<<push_back_point.z<<std::endl;
	////							std::cout<<"correspondences_inliers[i]"<<correspondences_inliers[i]<<std::endl;
	//			cvLine( imgadd, cvPoint(kpts[0][matches_popcount[correspondences_inliers[i]].queryIdx].pt.x, kpts[0][matches_popcount[correspondences_inliers[i]].queryIdx].pt.y),
	//					cvPoint(kpts[1][matches_popcount[correspondences_inliers[i]].trainIdx].pt.x, 480+kpts[1][matches_popcount[correspondences_inliers[i]].trainIdx].pt.y), colors[6] );
	//		}
	//	}
}




void EXTRACT::matchFeature(cv::Mat &dtors0,cv::Mat&dtors1,	vector<cv::DMatch> &matches)
{
	{
		//		std::cout<<"size of dtors0"<<dtors[0].size<<std::endl;
		//		std::cout<<"size of kpts0"<<kpts[0].size()<<std::endl;
		//		std::cout<<"before match"<<std::endl;
		//		std::cout<<"size of matches after clear"<<matches_popcount.size()<<std::endl;
		matcher_popcount.match(dtors0, dtors1, matches);
		//		std::cout<<"size of matches after .match"<<matches_popcount.size()<<std::endl;
		//		for(uint i=0; i<matches_popcount.size();i++)
		//			std::cout<<"matched from"<<matches_popcount.at(i).queryIdx<<"to"<<matches_popcount.at(i).trainIdx<<std::endl;

		//		std::cout<<"after match"<<std::endl;
		//		std::cout<<"after size of dtors0"<<dtors[0].size<<std::endl;
		//		std::cout<<"size of kpts0"<<kpts[0].size()<<std::endl;
	}
}

EXTRACT::EXTRACT(bool displ,float thresh, int iterations, int minimal_inliers, int keyframe_inliers, bool time, bool slam,int ignored, int near_keyframe_inliers, int swaps)
{
	take_vicon_z=swaps;
	take_initial_vicon=false;
	take_vicon=false;
	reset_map=false;

	notcopied=1;


	//	keyframeforreprojection=descFact;
	swap_counter=0;
	number_of_swaps=swaps;
	slammed=0;
	nearest_keyframe_inliers=near_keyframe_inliers;
	//SBA stuff
	//	maxx=640;
	//	maxy=480;
	//	project_counter=0;

	//	   data: [ 5.3348048003299175e+002, 0., 3.0105335457784764e+002, 0.,
	//	       5.3278657977389139e+002, 2.5844968495210247e+002, 0., 0., 1. ]

	//	   	cam_params.fx = 525; // Focal length in x
	//	   	cam_params.fy = 525; // Focal length in y
	//	   	cam_params.cx = 319.5; // X position of principal point
	//	   	cam_params.cy = 239.5; // Y position of principal point
	//	   	cam_params.tx = 0;   // Baseline (no baseline since this is monocular)

	//		cam_params.fx = 533;//48048003299175e+002; // Focal length in x
	//		cam_params.fy = 528;//8657977389139e+002; // Focal length in y
	//		cam_params.cx = 301;//05335457784764e+002; // X position of principal point
	//		cam_params.cy = 258;//44968495210247e+002; // Y position of principal point
	//		cam_params.tx = 0;   // Baseline (no baseline since this is monocular)



	numberOfIgnoredKeyframes=ignored;
	actual_keyframe=0;
	KeyFramePointClouds.header.frame_id="/pgraph";
	compute_counter=0;
	counter=0;
	computed=0;
	doSLAM=slam;
	showTime=time;
	transformation_at_least_once_computed=false;
	transformation_at_least_twice_computed=false;
	colored_pointcloud=false;

	imgadd=cvCreateImage( cvSize(640, 960), IPL_DEPTH_8U, 3 );
	track_reprojection_error=cvCreateImage( cvSize(640, 480), IPL_DEPTH_8U, 3 );


	//	keyTrans=Eigen::Matrix4f::Identity();

	next_keyframe=false;

	cv_image[0]=cvCreateImage(cvSize(640,480),8,3);
	cv_image[1]=cvCreateImage(cvSize(640,480),8,3);

	callback_image=cvCreateImage(cvSize(640,480),8,3);

	doICP=false;
	pFile = fopen ("myfile.txt","w");
	fprintf (pFile, "Good Correspondences, x,y,z,rotx,roty,rotz,rotw\n");


	averageNumberOfCorrespondences=0;
	averageTime=0;
	min_inliers=minimal_inliers;
	next_keyframe_inlier=keyframe_inliers;
	countOfBadCorrespondences=0;

	feature_type=4;

	ransac_acc=thresh;
	ransac_it=iterations;

	trans_vec.Zero();
	trans_vec_tmp.Zero();
	transformOld=Eigen::Matrix4f::Identity();
	transformGes=Eigen::Matrix4f::Identity();
	rot_matrix=Eigen::Matrix3f::Identity();
	compute_transform_success=1;




	FeaturePointCloud[0].header.frame_id=std::string("/openni_depth_optical_frame");
	FeaturePointCloud[0].is_dense = false;
	FeaturePointCloud[1].header.frame_id=std::string("/openni_depth_optical_frame");//"/openni_depth_optical_frame";
	FeaturePointCloud[1].is_dense = false;

	if(feature_type==1)
		detector = new cv::GridAdaptedFeatureDetector(new cv::FastFeatureDetector,500);
	if(feature_type==2)
		detector = new cv::GridAdaptedFeatureDetector(new cv::SurfFeatureDetector,500);
	if(feature_type==3)
		detector = new cv::GridAdaptedFeatureDetector(new cv::SiftFeatureDetector);
	if(feature_type==4)
		detector = new cv::GridAdaptedFeatureDetector(new cv::FastFeatureDetector,500);

	typedef cv::CalonderDescriptorExtractor<float> Calonder;
	if(feature_type==1)
		extractor= new Calonder("/home/benema/ros_tutorials/beginner_tutorials/src/calonder.rtc");
	if(feature_type==2)
		extractor=new cv::SurfDescriptorExtractor;
	if(feature_type==3)
		extractor=new cv::SiftDescriptorExtractor;
	if(feature_type==4)
		extractor=new cv::BriefDescriptorExtractor;

	//	std::cout<<"nach rtc"<<std::endl;


	callback_counter=0;

	showDisplay=displ;
	called=0;
	ros::NodeHandle n;


	//time the incoming rg and cloud msgs
	message_filters::Subscriber<PointCloud> pointCloud_sub(n, depth_topic, 1);
	message_filters::Subscriber<sensor_msgs::Image> rgbImage_sub(n, "/camera/rgb/image_color", 1);
	typedef message_filters::sync_policies::ApproximateTime<PointCloud, sensor_msgs::Image> MySyncPolicy;
	// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pointCloud_sub, rgbImage_sub);
	called_first_time=true;

	/*ROS PUBLISHERS */
	//for visualization
	path_pub=n.advertise<nav_msgs::Path>("mainSLAM/path",1);
	//for visualization
	cameraPose_pub=n.advertise<geometry_msgs::PoseStamped>("/mainSLAM/cameraPose",1);
	bodyPoseStamped_pub=n.advertise<geometry_msgs::PoseStamped>("/toMAVLINK/bodyPoseStamped",1);
	KeyFramePoints=n.advertise<PointCloud>("mainSLAM/keyframepoints",1);
	KeyFrameMarkers=n.advertise<nav_msgs::Odometry>("/mainSLAM/keyframemarkers",1);
	SeenKeyFramePoints=n.advertise<PointCloud>("/mainSLAM/seenkeyframepoints",1);
	//SBA
	//	transformedCloud=n.advertise<PointCloud>("mainSLAM/transformed",1);
	//SBA
	//	cam_marker_pub = n.advertise<visualization_msgs::Marker>("/mainSLAM/cameras", 1);

	//	camSBA_marker_pub = n.advertise<visualization_msgs::Marker>("/mainSLAM/camerasSBA", 1);
	//	pointSBA_marker_pub = n.advertise<visualization_msgs::Marker>("/mainSLAM/pointsSBA", 1);
	//	camSBA_marker_pub_preSBA = n.advertise<visualization_msgs::Marker>("/mainSLAM/cameras_preSBA", 1);
	//	pointSBA_marker_pub_preSBA = n.advertise<visualization_msgs::Marker>("/mainSLAM/points_preSBA", 1);

	//to fly
	imuSubscriber = n.subscribe ("/fromMAVLINK/Imu",  1, &EXTRACT::imuCallback,  this);
	viconSubscriber= n.subscribe("/fromMAVLINK/Vicon",1,&EXTRACT::viconCallback,this);
	commandSubscriber= n.subscribe("/fromMAVLINK/COMMAND",1,&EXTRACT::commandCallback,this);


	sync.registerCallback(&EXTRACT::callback,this);
	//	ros::spin();

	ros::AsyncSpinner spinner(0);
	spinner.start();
	while(ros::ok())
	{

		if(called==1)
		{
			if(reset_map)
					{

						std::cout<<"resetting map!!!"<<std::endl;
						counter=0;
						next_keyframe=false;
						reset_map=false;
						KeyframeDataVector.clear();
						path.poses.clear();
						notcopied=true;
						callback_counter=0;
//						called=0;
						actual_keyframe=0;
						compute_counter=0;
						computed=0;
						transformation_at_least_once_computed=false;
						transformation_at_least_twice_computed=false;
						colored_pointcloud=false;


						averageNumberOfCorrespondences=0;
						averageTime=0;

						countOfBadCorrespondences=0;

						transformOld=vicontransform;


						compute_transform_success=1;



						called_first_time=true;
						take_vicon=true;

						take_initial_vicon=true;



					}
			else{
			//			ros::spinOnce();

			if(showTime)
			{
				start = clock();
				std::cout<<"untill callback called again \t:"<< double((double(start)-double(finish))/CLOCKS_PER_SEC)<<std::endl;
			}
			if(showTime)
				start_time2=clock();
			if(called_first_time)
				counter=0;
			else
				counter=1;

			std::cout<<"counter:"<<counter<<std::endl;

			//			std::cout<<"counter and calledfirst"<<counter<<",<"<<called_first_time<<std::endl;
			cvCopy(callback_image,cv_image[counter]);
			kinectCloud[counter]=callbackCloud;
			kinectCloud[counter].header.frame_id="/pgraph";
			called=0;

			if(showTime)
				time_t start_time=clock();

			{
				//		        if( m.refcount )
				//		            CV_XADD(m.refcount, 1);
				//		        release();
				//		        flags = m.flags;
				//		        if( dims <= 2 && m.dims <= 2 )
				//		        {
				//		            dims = m.dims;
				//		            rows = m.rows;
				//		            cols = m.cols;
				//		            step[0] = m.step[0];
				//		            step[1] = m.step[1];
				//		        }
				//		        else
				//		            copySize(m);
				//		        data = m.data;
				//		        datastart = m.datastart;
				//		        dataend = m.dataend;
				//		        datalimit = m.datalimit;
				//		        refcount = m.refcount;
				//		        allocator = m.allocator;

				PointCloud tmpCloud;
				tmpCloud.header.frame_id="/pgraph";
				std::vector<cv::KeyPoint> kpts_tmp;



				Point tmpPoint;

				cv::Mat dtorstmp;
				detector->detect(cv_image[counter],kpts[counter]);
				extractor->compute(cv_image[counter],kpts[counter],dtors[counter]);
				dtorstmp.flags=dtors[counter].flags;
				dtorstmp.dims=dtors[counter].dims;
				dtorstmp.cols=dtors[counter].cols;
				dtorstmp.step[0]=dtors[counter].step[0];
				dtorstmp.step[1]=dtors[counter].step[1];
				//				if( dtorstmp.refcount )
				//				            CV_XADD(detorstmp.refcount, 1);
				//				        flags = m.flags;
				//				        if( dims <= 2 && m.dims <= 2 )
				//				        {
				//				            dims = m.dims;
				//				            rows = m.rows;
				//				            cols = m.cols;
				//				            step[0] = m.step[0];
				//				            step[1] = m.step[1]
				//				//				std::cout<<"testsize: kpts[counter]: "<<kpts[counter].size()<<std::endl;


				for(uint i=0;i<kpts[counter].size();i++)
				{
					//						std::cout<<"point at"<<kpts[counter].at(i).pt.x<<","<<kpts[counter].at(i).pt.y<<std::endl;
					if(	pcl_isfinite (kinectCloud[counter].at(kpts[counter].at(i).pt.x,kpts[counter].at(i).pt.y).x) &&
							pcl_isfinite (kinectCloud[counter].at(kpts[counter].at(i).pt.x,kpts[counter].at(i).pt.y).y) &&
							pcl_isfinite (kinectCloud[counter].at(kpts[counter].at(i).pt.x,kpts[counter].at(i).pt.y).z))
					{
						//						tmpPoint.x=kinectCloud[counter].at(kpts[counter].at(i).pt.x,kpts[counter].at(i).pt.y).x;
						//						tmpPoint.y=kinectCloud[counter].at(kpts[counter].at(i).pt.x,kpts[counter].at(i).pt.y).y;
						//						tmpPoint.z=kinectCloud[counter].at(kpts[counter].at(i).pt.x,kpts[counter].at(i).pt.y).z;
						tmpPoint=kinectCloud[counter].at(kpts[counter].at(i).pt.x,kpts[counter].at(i).pt.y);
						//						std::cout<<"pushback0:"<<i<<"pointx:"<<tmpPoint.x<<std::endl;
						//						std::cout<<"pushback0:"<<i<<"pointy:"<<tmpPoint.y<<std::endl;
						//						std::cout<<"pushback0:"<<i<<"pointz:"<<tmpPoint.z<<std::endl;
						//						FeatureTmp[1].push_back(FrameData[1].Points.points[matches_popcount[i].trainIdx]);
						//						push_back_point.x=kinectCloud[0].at(kpts[0][matches_popcount[i].queryIdx].pt.x,kpts[0][matches_popcount[i].queryIdx].pt.y).x;
						dtorstmp.push_back(dtors[counter].row(i));
						kpts_tmp.push_back(kpts[counter].at(i));
						tmpCloud.push_back(tmpPoint);

					}
				}
				dtors[counter]=dtorstmp;
				//				std::cout<<"testsize: kpts_tmp: "<<kpts_tmp.size()<<std::endl;
				kpts[counter]=kpts_tmp;
				//				std::cout<<"testsize: kpts[counter]: "<<kpts[counter].size()<<std::endl;
				//				std::cout<<"testsize: tmpCloud"<<tmpCloud.size()<<std::endl;
				//				extractor->compute(cv_image[counter],kpts[counter],dtors[counter]);
				//				std::cout<<"testsize: kpts[counter] after extractor: "<<kpts[counter].size()<<std::endl;
				//				FrameData[counter].Keypoint=kpts[counter];
				FrameData[counter].Points=tmpCloud;
				FrameData[counter].Descriptor=dtors[counter];
				FrameData[counter].Keypoints=kpts[counter];
				FrameData[counter].KinectCloud=kinectCloud[counter];
				std::cout<<"counter:"<<counter<<std::endl;
				std::cout<<"take_initial_vicon:"<<take_initial_vicon<<std::endl;
				if(counter==0)
				{
					if(take_initial_vicon)
					{
						std::cout<<"vicontransform"<<vicontransform<<std::endl;
						FrameData[counter].Transformation=vicontransform;//imuRot;
						std::cout<<"copying 0 if"<<std::endl;

					}
					else
					{
//						while(notcopied)
//							cvWaitKey(30);
						std::cout<<"vicontransform"<<vicontransform<<std::endl;

						std::cout<<"copying 0 esle"<<std::endl;
						FrameData[counter].Transformation=vicontransform;//;Eigen::Matrix4f::Identity();//imuRot;
					}
					KeyframeDataVector.push_back(FrameData[counter]);
					PointCloud tmp;
					tmp.header.frame_id="/pgraph";
					pcl::transformPointCloud(KeyframeDataVector.at(KeyframeDataVector.size()-1).Points,tmp,KeyframeDataVector.at(KeyframeDataVector.size()-1).Transformation);
					KeyFramePointClouds+=tmp;
					Eigen::Quaternion<float> quat_test(KeyframeDataVector.at(KeyframeDataVector.size()-1).Transformation.block<3,3>(0,0));
					trans_vec_keyframe=KeyframeDataVector.at(KeyframeDataVector.size()-1).Transformation.block<3,1>(0,3);
					quat_rot_keyframe=quat_test;
					//Publish Camera poses for SBA
					//					KeyFrameMarker.pose.pose.orientation.w=quat_rot_keyframe.w();
					//					KeyFrameMarker.pose.pose.orientation.x=quat_rot_keyframe.x();//+cos(M_PI/2.0);
					//					KeyFrameMarker.pose.pose.orientation.y=quat_rot_keyframe.y();//-cos(M_PI/2.0);
					//					KeyFrameMarker.pose.pose.orientation.z=quat_rot_keyframe.z();//+cos(M_PI/2.0);
					//					KeyFrameMarker.pose.pose.position.x=trans_vec_keyframe[0];
					//					KeyFrameMarker.pose.pose.position.y=trans_vec_keyframe[1];
					//					KeyFrameMarker.pose.pose.position.z=trans_vec_keyframe[2];

					KeyFrameMarker.header.frame_id="/pgraph";//kinectCloud[0].header.frame_id;
					KeyFrameMarker.header.stamp=ros::Time::now();
					//if(compute_transform_success)
					KeyFrameMarkers.publish(KeyFrameMarker);
					//					std::cout<<"size of keyframedatavector"<<KeyframeDataVector.size()<<std::endl;
					//					if(doSLAM)
					//					{
					//						SBAmap.Descriptor.push_back(KeyframeDataVector.at(KeyframeDataVector.size()-1).Descriptor);
					//						//						SBAmap.Keypoints=KeyframeDataVector.at(KeyframeDataVector.size()-1).Keypoints;
					//						SBAmap.Points.push_back(KeyframeDataVector.at(KeyframeDataVector.size()-1).Points);
					//
					//					}
				}
				if(counter)
					compute_counter++;
			}
			called_first_time=false;

			if(showTime)
			{
				time_t end_time=clock();
				std::cout<<"time for detection and extraction:\t"<<(float(end_time)-float(start_time))/CLOCKS_PER_SEC<<std::endl;

				end_time2=clock();
				std::cout<<"time for detection and extraction and copy:\t"<<(float(end_time2)-float(start_time2))/CLOCKS_PER_SEC<<std::endl;

			}
			{
				if(counter==1)
				{
					//					std::cout<<"beforeshow"<<std::endl;
					if(showDisplay)
					{
						cvSetImageROI( imgadd,cvRect( 0, 0,cv_image[0]->width, cv_image[0]->height ) );
						cvCopy(cv_image[0],imgadd);
						cvResetImageROI(imgadd);
						cvSetImageROI( imgadd,cvRect( 0, cv_image[0]->height,cv_image[0]->width, cv_image[0]->height ) );
						cvCopy(cv_image[1],imgadd);
						cvResetImageROI(imgadd);
					}
					if(showTime)
						start_time=clock();
					//extract the features
					//					std::cout<<"beforematch"<<std::endl;

					if(actual_keyframe==(KeyframeDataVector.size()-1))
						matchFeature(dtors[0],dtors[1],matches_popcount);
					else
						matchFeature(KeyframeDataVector.at(actual_keyframe).Descriptor,dtors[1],matches_popcount);

					std::cout<<"size of matches_popcount"<<matches_popcount.size()<<std::endl;

					if(showTime)
					{
						end_time=clock();
						std::cout<<"time for match Feature:\t"<<(float(end_time)-float(start_time))/CLOCKS_PER_SEC<<std::endl;
					}

					if(showTime)
						start_time=clock();
					//do RANSAC and compute the transforamtion
					RANSACandTransformation();
					if(showTime)
					{
						end_time=clock();
						std::cout<<"time for RANSAC and Tranformation:\t"<<(float(end_time)-float(start_time))/CLOCKS_PER_SEC<<std::endl;
					}
					//					if(doSLAM)
					//					{
					//						if(showTime)
					//							start_time=clock();
					//						//transfomr the pointclouds
					//						transformPointcloud();
					//						if(showTime)
					//						{
					//							end_time=clock();
					//							std::cout<<"time for transfomringpointcloud:\t"<<(float(end_time)-float(start_time))/CLOCKS_PER_SEC<<std::endl;
					//						}
					//					}
					if(doSLAM&&slammed==0)
					{
						//						slammed=1;
						//						//Publish Camera poses for SBA
						//						camera_pos.pose.orientation.x=quat_rot.x()/quat_rot.w();
						//						camera_pos.pose.orientation.y=quat_rot.y()/quat_rot.w();
						//						camera_pos.pose.orientation.z=quat_rot.z()/quat_rot.w();
						//						camera_pos.header.frame_id="/pgraph";
						//						ros::Time tstamp=ros::Time::now();
						//						camera_pos.header.stamp=tstamp;
						//						camera_pos.type=visualization_msgs::Marker::CUBE;
						//						camera_pos.scale.x=1;
						//						camera_pos.scale.y=1;
						//						camera_pos.scale.z=1;
						//						camera_pos.pose.position.x=trans_vec[0];
						//						camera_pos.pose.position.y=trans_vec[1];
						//						camera_pos.pose.position.z=trans_vec[2];
						//						camera_pos.ns = "my_namespace";
						//						camera_pos.id = 0;
						//						camera_pos.color.a = 1.0f;
						//						camera_pos.color.r = 0.0f;
						//						camera_pos.color.g = 1.0f;
						//						camera_pos.color.b = 0.0f;
						//						cam_marker_pub.publish(camera_pos);
						//						transformPointcloud();
						////						transformedCloud.publish(kinectCloud[0]);
						//						std::cout<<"pulished the slam stuff"<<std::endl;


					}
					publishEverything();
					averageTime+=whole_time;
					if(showTime)
						std::cout<<"thewholethingtakes:"<<whole_time<<"averagetime:"<<averageTime/float(compute_counter)<<std::endl;




					if(showTime)
						start_time=clock();


					if(next_keyframe)
					{
						swap();

					}
					if(showDisplay)
					{
						cvShowImage("RGB Image 1", imgadd);
						//cvShowImage("RGB Image 0", cv_image[0]);
						cvWaitKey(30);
						//				fprintf (pFile, "%d,%f,%f,%f,%f,%f,%f\n",correspondences_source_good.size(),trans_vec[0],trans_vec[1],trans_vec[2],quat_rot.x(),quat_rot.y(),quat_rot.z(),quat_rot.w());

					}


					//		std::cout<<"vor calback ende briefmapdesc"<<briefMapDescriptors->total<<"briefframedesc"<<briefFrameDescriptors->total<<std::endl;
					if(showTime)
					{
						end_time=clock();
						std::cout<<"time for swapping:\t"<<(float(end_time)-float(start_time))/CLOCKS_PER_SEC<<std::endl;
					}

				}
				if(showTime)
				{
					finish = clock();
					whole_time = (double(finish)-double(start))/CLOCKS_PER_SEC;
				}
			}
			//ROS_INFO("waiting for next image... bad corresp. until now:%d",countOfBadCorrespondences);
		}
		}

	}

	spinner.stop();
	//	cvReleaseImage(&callback_image[0]);
	//	cvReleaseImage(&callback_image[1]);
	cvReleaseImage(&imgadd);
	cvReleaseImage(&cv_image[0]);
	cvReleaseImage(&cv_image[1]);
	cvReleaseImage(&track_reprojection_error);

}


void EXTRACT::swap()
{


	std::cout<<"next_keyframe"<<std::endl;
	FrameData[0].Descriptor=FrameData[1].Descriptor;
	FrameData[0].Points=FrameData[1].Points;
	FrameData[1].matches_backward=matches_popcount;
	FrameData[1].correspondences_backward=correspondences_inliers;

	KeyframeDataVector.push_back(FrameData[1]);
	actual_keyframe=KeyframeDataVector.size()-1;
	std::cout<<"before quaternion"<<std::endl;
	Eigen::Quaternion<float> quat_test(KeyframeDataVector.at(KeyframeDataVector.size()-1).Transformation.block<3,3>(0,0));
	std::cout<<"after quat"<<std::endl;
	trans_vec_keyframe=KeyframeDataVector.at(KeyframeDataVector.size()-1).Transformation.block<3,1>(0,3);
	quat_rot_keyframe=quat_test;
	//Publish Camera poses for SBA
	KeyFrameMarker.pose.pose.orientation.w=quat_rot_keyframe.w();
	KeyFrameMarker.pose.pose.orientation.x=quat_rot_keyframe.x();//+cos(M_PI/2.0);
	KeyFrameMarker.pose.pose.orientation.y=quat_rot_keyframe.y();//-cos(M_PI/2.0);
	KeyFrameMarker.pose.pose.orientation.z=quat_rot_keyframe.z();//+cos(M_PI/2.0);
	KeyFrameMarker.pose.pose.position.x=trans_vec_keyframe[0];
	KeyFrameMarker.pose.pose.position.y=trans_vec_keyframe[1];
	KeyFrameMarker.pose.pose.position.z=trans_vec_keyframe[2];

	KeyFrameMarker.header.frame_id="/pgraph";//kinectCloud[0].header.frame_id;
	KeyFrameMarker.header.stamp=ros::Time::now();
	//if(compute_transform_success)
	KeyFrameMarkers.publish(KeyFrameMarker);
	//	std::cout<<"size of keyframedatavector"<<KeyframeDataVector.size()<<std::endl;
	//	std::cout<<"dist between two keyframes:"<<KeyframeDataVector.at(KeyframeDataVector.size()-2).Transformation.inverse()*KeyframeDataVector.at(KeyframeDataVector.size()-1).Transformation<<std::endl;

	kpts[0]=kpts[1];
	dtors[0]=dtors[1];

	kinectCloud[0]=kinectCloud[1];
	cvCopy(cv_image[1],cv_image[0]);

	//	findNearestKeyframetoLastandComputeTransformation(KeyframeDataVector.at(KeyframeDataVector.size()-1));
	FrameData[0].Transformation=FrameData[1].Transformation;
	PointCloud tmp;
	tmp.header.frame_id="/pgraph";
	//	pcl::transformPointCloud(KeyframeDataVector.at(KeyframeDataVector.size()-2).Points,tmp,KeyframeDataVector.at(KeyframeDataVector.size()-2).Transformation);
	pcl::transformPointCloud(KeyframeDataVector.at(KeyframeDataVector.size()-1).Points,tmp,KeyframeDataVector.at(KeyframeDataVector.size()-1).Transformation);
	KeyFramePointClouds+=tmp;


	//	std::cout<<"transform old after correction:"<<std::endl<<transformOld<<std::endl;
	std::cout<<"doslam:"<<doSLAM<<std::endl;
	if(doSLAM)
	{
		//		//		std::cout<<"size of correspond"<<correspondences_outliers.size()<<std::endl;
		//		//		std::cout<<"size of matches"<<matches_popcount.size()<<std::endl;
		//		//		std::cout<<"size of tmp"<<tmp.size()<<std::endl;
		//		//compute sbadata
		//		PointCloud tmpsbapointcloud;
		//		tmpsbapointcloud.header.frame_id="/pgraph";
		//		cv::Mat dtorstmp;
		//		dtorstmp.flags=dtors[counter].flags;
		//		dtorstmp.dims=dtors[counter].dims;
		//		dtorstmp.cols=dtors[counter].cols;
		//		dtorstmp.step[0]=dtors[counter].step[0];
		//		dtorstmp.step[1]=dtors[counter].step[1];
		////		std::cout<<"before pushing back inliers size of keyframe"<<KeyframeDataVector.size()<<std::endl;
		//
		//		struct mapPoint tmp;
		//		std::vector<int> tmp_camera (3);
		//		struct FrameData OldData=KeyframeDataVector.at(KeyframeDataVector.size()-2);
		//		struct FrameData NewData=KeyframeDataVector.at(KeyframeDataVector.size()-1);
		//
		//		pcl::transformPointCloud(OldData.Points,OldData.Points,OldData.Transformation);
		//		pcl::transformPointCloud(NewData.Points,NewData.Points,NewData.Transformation);
		//
		//		if(KeyframeDataVector.size()==2)
		//			for(uint q=0;q<correspondences_inliers.size();q++)
		//			{
		//				tmp.cameras.clear();
		//				//				std::cout<<"1"<<std::endl;
		//				//				tmp.PointXYZ=KeyframeDataVector.at(second_last).Points.points.at(KeyframeDataVector.at(KeyframeDataVector.size()-2)).matches_backward.at(correspondences_inliers.at(q).queryIdx);
		//				tmp.PointXYZ=OldData.Points.points.at(NewData.matches_backward.at(NewData.correspondences_backward.at(q)).queryIdx);
		//				tmp_camera[0]=0;
		//				tmp_camera[1]=OldData.Keypoints.at(NewData.matches_backward.at(NewData.correspondences_backward.at(q)).queryIdx).pt.x;
		//				tmp_camera[2]=OldData.Keypoints.at(NewData.matches_backward.at(NewData.correspondences_backward.at(q)).queryIdx).pt.y;
		//
		//				circle.x=tmp_camera[1];
		//				circle.y=tmp_camera[2];
		//				cvCircle( imgadd, circle, 3, colors[5], 3, 8, 0 );
		//
		//				tmp.cameras.push_back(tmp_camera);
		//				tmp_camera[0]=1;
		//				tmp_camera[1]=NewData.Keypoints.at(NewData.matches_backward.at(NewData.correspondences_backward.at(q)).trainIdx).pt.x;
		//				tmp_camera[2]=NewData.Keypoints.at(NewData.matches_backward.at(NewData.correspondences_backward.at(q)).trainIdx).pt.y;
		//				circle.x=tmp_camera[1];
		//				circle.y=tmp_camera[2]+480;
		//				cvCircle( imgadd, circle, 3, colors[5], 3, 8, 0 );
		//				tmp.cameras.push_back(tmp_camera);
		//				tmp.identifier=NewData.matches_backward.at(NewData.correspondences_backward.at(q)).trainIdx;
		//				SBAPoint.push_back(tmp);
		//			}
		//
		//		int sizeofSBAPoint=SBAPoint.size();
		//		std::vector<int> updated_points(sizeofSBAPoint,0);
		////		std::cout<<"updatedpoints size"<<updated_points.size();
		//		//		std::cout<<"sbapoint.size()"<<SBAPoint.size()<<std::endl;
		//		if(KeyframeDataVector.size()>2)
		//		{
		////			std::cout<<"2"<<std::endl;
		//
		//
		//			for(uint w=0;w<NewData.correspondences_backward.size();w++)
		//			{
		//				bool new_inlier=true;
		//				for(uint q=0;q<sizeofSBAPoint;q++)
		//					if(NewData.matches_backward.at(NewData.correspondences_backward.at(w)).queryIdx==SBAPoint.at(q).identifier)
		//					{
		//						if(SBAPoint.at(q).cameras.at(SBAPoint.at(q).cameras.size()-1).at(0)!=KeyframeDataVector.size()-1)
		//						{
		//							tmp.cameras.clear();
		//							tmp_camera[0]=KeyframeDataVector.size()-1;
		//							tmp_camera[1]=NewData.Keypoints.at(NewData.matches_backward.at(NewData.correspondences_backward.at(w)).trainIdx).pt.x;
		//							tmp_camera[2]=NewData.Keypoints.at(NewData.matches_backward.at(NewData.correspondences_backward.at(w)).trainIdx).pt.y;
		//							circle.x=tmp_camera[1];
		//							circle.y=tmp_camera[2]+480;
		//							cvCircle( imgadd, circle, 3, colors[5], 3, 8, 0 );
		//							SBAPoint.at(q).cameras.push_back(tmp_camera);
		//							SBAPoint.at(q).identifier=NewData.matches_backward.at(NewData.correspondences_backward.at(w)).trainIdx;
		//							updated_points.at(q)=1;
		//						}
		//						new_inlier=false;
		//
		//					}
		////				std::cout<<"3"<<std::endl;
		//
		//				if(new_inlier)
		//				{
		//					tmp.cameras.clear();
		//					tmp_camera[0]=KeyframeDataVector.size()-2;
		////					std::cout<<"herlor0"<<std::endl;
		////					std::cout<<"NewData.correspondences_backward.at(w)"<<NewData.correspondences_backward.at(w)<<std::endl;
		////					std::cout<<"NewData.matches_backward.at(NewData.correspondences_backward.at(w)).queryIdx"<<NewData.matches_backward.at(NewData.correspondences_backward.at(w)).queryIdx<<std::endl;
		////					std::cout<<"OldData.Keypoints.at(NewData.matches_backward.at(NewData.correspondences_backward.at(w)).queryIdx).pt.x"<<std::endl;
		//					tmp_camera[1]=OldData.Keypoints.at(NewData.matches_backward.at(NewData.correspondences_backward.at(w)).queryIdx).pt.x;
		//					tmp_camera[2]=OldData.Keypoints.at(NewData.matches_backward.at(NewData.correspondences_backward.at(w)).queryIdx).pt.y;
		//					circle.x=tmp_camera[1];
		//					circle.y=tmp_camera[2];
		//					cvCircle( imgadd, circle, 3, colors[5], 3, 8, 0 );
		//					tmp.cameras.push_back(tmp_camera);
		//					tmp_camera[0]=KeyframeDataVector.size()-1;
		////					std::cout<<"herlor0.5"<<std::endl;
		//
		//					tmp_camera[1]=NewData.Keypoints.at(NewData.matches_backward.at(NewData.correspondences_backward.at(w)).trainIdx).pt.x;
		//					tmp_camera[2]=NewData.Keypoints.at(NewData.matches_backward.at(NewData.correspondences_backward.at(w)).trainIdx).pt.y;
		//					circle.x=tmp_camera[1];
		//					circle.y=tmp_camera[2]+480;
		//					cvCircle( imgadd, circle, 3, colors[5], 3, 8, 0 );
		//					tmp.cameras.push_back(tmp_camera);
		//					tmp.PointXYZ=OldData.Points.points.at(NewData.matches_backward.at(NewData.correspondences_backward.at(w)).queryIdx);
		////					std::cout<<"herlor"<<std::endl;
		//					tmp.identifier=NewData.matches_backward.at(NewData.correspondences_backward.at(w)).trainIdx;
		////					std::cout<<"herlor2"<<std::endl;
		//
		//					SBAPoint.push_back(tmp);
		//				}
		//
		////				std::cout<<"4"<<std::endl;
		//
		//			}
		//
		//			std::cout<<"size of sba"<<SBAPoint.size()<<std::endl;
		//			//			std::cout<<"size of updated"<<updated_points.size()<<std::endl;
		//
		//			for(uint a=0;a<updated_points.size();a++)
		//				if(updated_points.at(a)==0)
		//				{
		//					//					std::cout<<"a:"<<a<<std::endl;
		//					SBAPoint.at(a).identifier=100000;
		//				}
		//
		//			//			std::cout<<"5"<<std::endl;
		//
		//		}
		//
		////		PointCloud SBAPointCloud;
		////		SBAPointCloud.header.frame_id="/pgraph";
		////
		////		for(uint t=0;t<SBAPoint.size();t++)
		////		{
		////			SBAPointCloud.points.push_back(SBAPoint.at(t).PointXYZ);
		////			if(t%50==0)
		////			{
		////				if(SBAPoint.at(t).cameras.size()>2)
		////				{
		////					std::cout<<"sbapoint.cameras:"<<std::endl;
		////					for(uint k=0;k<SBAPoint.at(t).cameras.size();k++)
		////					{
		////						std::cout<<"k:"<<k<<std::endl;
		////						std::cout<<"cameras.at(k).at(0)"<<SBAPoint.at(t).cameras[k][0]<<std::endl;
		////					}
		////				}
		////				else
		////					std::cout<<"only two at t:"<<t<<std::endl;
		////
		////			}
		////		}
		////
		////		KeyFramePoints.publish(SBAPointCloud);
		////		std::cout<<"size of keyframes:"<<KeyframeDataVector.size()<<std::endl;
		//
		//
		//
		//
		//		//		//		else
		//		//			new_inliers=correspondences_inliers;
		//		//		std::cout<<"keyframedatavector.size():"<<KeyframeDataVector.size()<<std::endl;
		//		//		for(uint i=0;i<new_inliers.size();i++)
		//		//		{
		//		//			//			std::cout<<"corres.at"<<correspondences_outliers.at(i)<<std::endl;
		//		//			//			std::cout<<"heiho1"<<std::endl;
		//		//			tmpsbapointcloud.points.push_back(tmp.points.at(matches_popcount[new_inliers.at(i)].trainIdx));
		//		//			//			SBAmap.Points.points.push_back(KeyframeDataVector.at(KeyframeDataVector.size()-1).Points.points.at(matches_popcount[correspondences_outliers.at(i)].trainIdx));
		//		//			//			std::cout<<"heiho2"<<std::endl;
		//		//			dtorstmp.push_back(KeyframeDataVector.at(KeyframeDataVector.size()-1).Descriptor.row(matches_popcount[new_inliers.at(i)].trainIdx));
		//		//			//			std::cout<<"i:"<<i<<std::endl;
		//		//		}
		//		//
		//		//
		//		//		prev_corresp=correspondences_inliers;
		//		//		prev_matches=matches_popcount;
		//
		//
		//		//		SBAmap.Descriptor.push_back(dtorstmp);//KeyframeDataVector.at(KeyframeDataVector.size()-1).Descriptor);
		//		//		//			//		pcl::transformPointCloud(tmpsbapointcloud,tmpsbapointcloud,KeyframeDataVector.at(KeyframeDataVector.size()-1).Transformation);
		//		//		SBAmap.Points.push_back(tmpsbapointcloud);
		//		//					SBAmap.Descriptor.push_back(KeyframeDataVector.at(KeyframeDataVector.size()-1).Descriptor);//KeyframeDataVector.at(KeyframeDataVector.size()-1).Descriptor);
		//		//		pcl::transformPointCloud(tmpsbapointcloud,tmpsbapointcloud,KeyframeDataVector.at(KeyframeDataVector.size()-1).Transformation);
		//		//					SBAmap.Points.push_back(tmp);
		//
		//		//		for(uint i=0;i<correspondences_inliers.size();i++)
		//		//		{
		//		//			//			std::cout<<"corres.at"<<correspondences_outliers.at(i)<<std::endl;
		//		//			//			std::cout<<"heiho1"<<std::endl;
		//		//			SBAmap.Points.points.push_back(tmp.points.at(matches_popcount[correspondences_inliers.at(i)].trainIdx));
		//		////			SBAmap.Points.points.push_back(KeyframeDataVector.at(KeyframeDataVector.size()-1).Points.points.at(matches_popcount[correspondences_outliers.at(i)].trainIdx));
		//		//			//			std::cout<<"heiho2"<<std::endl;
		//		//			SBAmap.Descriptor.push_back(KeyframeDataVector.at(KeyframeDataVector.size()-1).Descriptor.row(matches_popcount[correspondences_outliers.at(i)].trainIdx));
		//		//			//			std::cout<<"i:"<<i<<std::endl;
		//		//		}
		//
		//		//Publish Camera poses for SBA
		//		camera_pos.pose.orientation.x=quat_rot.x()/quat_rot.w();
		//		camera_pos.pose.orientation.y=quat_rot.y()/quat_rot.w();
		//		camera_pos.pose.orientation.z=quat_rot.z()/quat_rot.w();
		//		camera_pos.header.frame_id="/pgraph";
		//		ros::Time tstamp=ros::Time::now();
		//		camera_pos.header.stamp=tstamp;
		//		camera_pos.type=visualization_msgs::Marker::CUBE;
		//		camera_pos.scale.x=1;
		//		camera_pos.scale.y=1;
		//		camera_pos.scale.z=1;
		//		camera_pos.pose.position.x=trans_vec[0];
		//		camera_pos.pose.position.y=trans_vec[1];
		//		camera_pos.pose.position.z=trans_vec[2];
		//		camera_pos.ns = "my_namespace";
		//		camera_pos.id = 0;
		//		camera_pos.color.a = 1.0f;
		//		camera_pos.color.r = 0.0f;
		//		camera_pos.color.g = 1.0f;
		//		camera_pos.color.b = 0.0f;
		//		cam_marker_pub.publish(camera_pos);
		//		transformPointcloud();
		////		transformedCloud.publish(kinectTransformedOld);
		//		std::cout<<"pulished the slam stuff"<<std::endl;
		//		swap_counter++;
		//
		//		if(swap_counter>number_of_swaps)
		//		{
		//			doSBAwithMap();
		//		}
		//
		//
	}

}

void EXTRACT::findNearestKeyframetoLastandComputeTransformation(struct FrameData& Last)
{
	if(KeyframeDataVector.size()>(numberOfIgnoredKeyframes))
	{
		Eigen::Vector3f tmp_dist;
		float small_dist=10000;
		uint keyframe_number=0;
		vector<cv::DMatch> matches;
		std::vector<int> corr_Keyframe;
		std::vector<int> corr_Keyframe_inliers;
		std::vector<int> corr_Keyframe_inliers_tmp;
		std::vector<int> corr_Keyframe_outliers;
		PointCloud Feature_Keyframe[2];
		Feature_Keyframe[0].header.frame_id="/pgraph";
		Feature_Keyframe[1].header.frame_id="/pgraph";

		//	std::cout<<"in nearest"<<std::endl;
		for(uint i=0;i<KeyframeDataVector.size()-(numberOfIgnoredKeyframes+1);i++)
		{
			tmp_dist=(KeyframeDataVector.at(i).Transformation.inverse()*Last.Transformation).block<3,1>(0,3);
			//		std::cout<<"tmp_dist"<<tmp_dist<<"tmp_distnorm:"<<tmp_dist.norm()<<std::endl;
			if(tmp_dist.norm()<small_dist)
			{
				small_dist=tmp_dist.norm();
				keyframe_number=i;
			}

		}

		//	std::cout<<"in 2"<<std::endl;

		if(small_dist!=10000)
			std::cout<<"nearest keyframe from: "<<KeyframeDataVector.size()-1<< "is:"<<keyframe_number<<"with dist:"<<small_dist<<std::endl;
		else
		{
			std::cout<<"no nearest keyframe except for last"<<std::endl;
			return;
		}

		matchFeature(KeyframeDataVector.at(keyframe_number).Descriptor,Last.Descriptor,matches);

		for (size_t i = 0; i < matches.size(); i++)
		{
			Feature_Keyframe[0].push_back(KeyframeDataVector.at(keyframe_number).Points.points[matches[i].queryIdx]);
			corr_Keyframe.push_back(i);
			Feature_Keyframe[1].push_back(Last.Points.points[matches[i].trainIdx]);

		}

		//RANSAC


		typedef pcl::SampleConsensusModelRegistration<Point>::Ptr SampleConsensusModelRegistrationPtr;

		SampleConsensusModelRegistrationPtr model;


		model.reset (new pcl::SampleConsensusModelRegistration<Point> (Feature_Keyframe[0].makeShared (), corr_Keyframe));
		// Pass the target_indices


		model->setInputTarget (Feature_Keyframe[1].makeShared(), corr_Keyframe);
		// Create a RANSAC model

		pcl::RandomSampleConsensus<Point> sac (model, ransac_acc);
		sac.setMaxIterations (ransac_it);


		// Compute the set of inliers

		if (!sac.computeModel (5))
		{
			corr_Keyframe_inliers = corr_Keyframe;
		}
		else
		{
			std::vector<int> inliers;
			// Get the inliers
			sac.getInliers (inliers);
			corr_Keyframe_inliers.resize (inliers.size ());
			// Copy just the inliers
			for (size_t i = 0; i < inliers.size (); ++i)
			{
				corr_Keyframe_inliers[i] = corr_Keyframe[inliers[i]];
			}
			//						std::vector<int> corr_Keyframe_in;
			//						int c=0;
			//						for (size_t i = 0; i < corr_Keyframe.size (); ++i)
			//						{
			//							if(i==inliers[c])
			//							{
			//								corr_Keyframe_in.push_back(corr_Keyframe[inliers[c]]);
			//								c++;
			//							}
			//							else
			//								corr_Keyframe_outliers.push_back(corr_Keyframe[i]);
			//						}
			//
			//
			//						std::cout<<"size of keyframeoriginal"<<corr_Keyframe_inliers.size()<<"size of keyframenew"<<corr_Keyframe_in.size()<<std::endl;
			//						for(uint i=0;i<corr_Keyframe_inliers.size();i++)
			//						{
			//							std::cout<<"keyframeor"<<corr_Keyframe_inliers.at(i)<<"keyframenew"<<corr_Keyframe_in.at(i)<<std::endl;
			//						}
			//						for(uint i=0;i<corr_Keyframe_outliers.size();i++)
			//							std::cout<<"outliers"<<corr_Keyframe_outliers.at(i)<<std::endl;

		}

		if(showDisplay)
		{
			ROS_ERROR("number of correspondences  after ransac with nearest keyframe %d",corr_Keyframe_inliers.size());
		}
		ransac_inliers=corr_Keyframe_inliers.size();

		//	std::cout<<"minransac keyfram"<<next_keyframe_inlier<<std::endl;
		//	std::cout<<"next keyframe:"<<next_keyframe<<std::endl;
		//Check whether we need next keyframe
		if(ransac_inliers>nearest_keyframe_inliers)
		{
			std::cout<<"computing new transform"<<std::endl;
			pcl::estimateRigidTransformationSVD(Feature_Keyframe[1],corr_Keyframe_inliers,Feature_Keyframe[0],corr_Keyframe_inliers,transformation_);
			std::cout<<"transformation between:\n"<<transformation_<<std::endl;
			transformOld=KeyframeDataVector.at(keyframe_number).Transformation*transformation_;
			next_keyframe=false;
			actual_keyframe=keyframe_number;
			FrameData[counter].Transformation=transformOld;
			//adding points to keyframe
			//			pcl::transformPointCloud(Last.Points,Last.Points,transformOld*Last.Transformation.inverse());
			Last.Transformation=transformOld;

			std::cout<<"new transform \n"<<transformOld<<std::endl;

		}
	}
	else
		return;




}


void EXTRACT::callback(const PointCloud::ConstPtr& pointCloud_ptr,const sensor_msgs::ImageConstPtr& image_ptr)
{
	if(showTime)
	{
		callback_time_start=clock();
		std::cout<<"time between callback"<<double((double(callback_time_start)-double(callback_time_end))/CLOCKS_PER_SEC)<<std::endl;
	}
	static sensor_msgs::CvBridge bridgeImage1;


	try
	{
		callbackCloud=*pointCloud_ptr;
		callback_image = bridgeImage1.imgMsgToCv(image_ptr, "bgr8");
	}
	catch (sensor_msgs::CvBridgeException error)
	{

		ROS_ERROR("error");

	}
	called=1;
	//	std::cout<<"in callback"<<std::endl;
	callback_counter++;


	//ROS_INFO("waiting for next image... bad corresp. until now:%d",countOfBadCorrespondences);
	if(showTime)
		callback_time_end=clock();

}


//void EXTRACT::ownSBARANSAC(struct FrameData &Data,	vector<cv::DMatch> &matches, std::vector<int> &corresKey,std::vector<int> &corresSBA,int keyframenumber,PointCloud &sbacloud)
//{
//	double err_ges=100000;
//	Eigen::Vector3f tmp_dist;
//	//	float small_dist=10000;
//	//	uint keyframe_number=0;
//	std::vector<int> corr_Keyframe;
//	std::vector<int> corr_sba;
//	std::vector<int> corr_Keyframe_inliers;
//	std::vector<int> corr_Keyframe_inliers_tmp;
//	std::vector<int> corr_Keyframe_outliers;
//	PointCloud Feature_Keyframe[2];
//	Feature_Keyframe[0].header.frame_id="/pgraph";
//	Feature_Keyframe[1].header.frame_id="/pgraph";
//
//	cv::Mat tmpsbadesc;
//	tmpsbadesc.flags=dtors[counter].flags;
//	tmpsbadesc.dims=dtors[counter].dims;
//	tmpsbadesc.cols=dtors[counter].cols;
//	tmpsbadesc.step[0]=dtors[counter].step[0];
//	tmpsbadesc.step[1]=dtors[counter].step[1];
//	PointCloud tmpsbacloud;
//	tmpsbacloud.header.frame_id="/pgraph";
//
//	for(uint k=0;k<SBAmap.Descriptor.size();k++)
//	{
//		if(k!=keyframenumber)
//		{
//			for(uint i=0;i<SBAmap.Descriptor.at(k).rows;i++)
//				tmpsbadesc.push_back(SBAmap.Descriptor.at(k).row(i));
//			tmpsbacloud+=SBAmap.Points.at(k);
//		}
//
//	}
//
//	sbacloud.header.frame_id="/pgraph";
//	sbacloud=tmpsbacloud;
//
//
//	std::cout<<"before matches"<<std::endl;
//	matchFeature(Data.Descriptor,tmpsbadesc,matches);
//
//	for (size_t i = 0; i < matches.size(); i++)
//	{
//		corr_Keyframe.push_back(matches[i].queryIdx);
//		corr_sba.push_back(matches[i].trainIdx);
//
//	}
//	int nmatch=matches.size();
//	PointCloud tmp;
//	tmp.header.frame_id="/pgraph";
//	pcl::transformPointCloud(Data.Points,tmp,Data.Transformation);
//
//
//	for(uint i=0;i<ransac_it;i++)
//	{
//		std::vector<int> correspondences_inliersKey_tmp;
//		std::vector<int> correspondences_inliersSBA_tmp;
//		Eigen::Matrix4f randomTrans;
//		float err_ges_temp=0;
//		int inl_tmp=0;
//		// find a candidate
//		int a=rand()%nmatch;
//		int b = a;
//		while (a==b)
//			b=rand()%nmatch;
//		int c = a;
//		while (a==c || b==c)
//			c=rand()%nmatch;
//
//		std::vector<int> rand_corr_key;
//		rand_corr_key.push_back(corr_Keyframe.at(a));
//		rand_corr_key.push_back(corr_Keyframe.at(b));
//		rand_corr_key.push_back(corr_Keyframe.at(c));
//		std::vector<int> rand_corr_sba;
//		rand_corr_sba.push_back(corr_sba.at(a));
//		rand_corr_sba.push_back(corr_sba.at(b));
//		rand_corr_sba.push_back(corr_sba.at(c));
//		//		rand_PointCloud[0].push_back(FeaturePointCloud[0].points.at(a));
//		//		rand_PointCloud[0].push_back(FeaturePointCloud[0].points.at(b));
//		//		rand_PointCloud[0].push_back(FeaturePointCloud[0].points.at(c));
//		//		rand_PointCloud[1].push_back(FeaturePointCloud[1].points.at(a));
//		//		rand_PointCloud[1].push_back(FeaturePointCloud[1].points.at(b));
//		//		rand_PointCloud[1].push_back(FeaturePointCloud[1].points.at(c));
//		//
//		//		pcl::estimateRigidTransformationSVD(rand_PointCloud[1],rand_corr,rand_PointCloud[0],rand_corr, randomTrans);
//		//
//		pcl::estimateRigidTransformationSVD(tmp,rand_corr_key,tmpsbacloud,rand_corr_sba, randomTrans);
//
//		Eigen::Vector3f Trand=randomTrans.block<3,1>(0,3);
//		Eigen::Matrix3f Rrand=randomTrans.block<3,3>(0,0);
//		//		correspondences_inliers.clear();
//		//				correspondences_inliers_tmp.clear();
//		// find inliers, based on image reprojection
//		for (int u=0; u<nmatch; u++)
//		{
//			if(u!=a&&u!=b&&u!=c)
//			{
//				Eigen::Vector3f pt(tmp.points.at(corr_Keyframe.at(u)).x,tmp.points.at(corr_Keyframe.at(u)).y,tmp.points.at(corr_Keyframe.at(u)).z);
//				Eigen::Vector3f ipt(tmpsbacloud.points.at(corr_sba.at(u)).x,tmpsbacloud.points.at(corr_sba.at(u)).y,tmpsbacloud.points.at(corr_sba.at(u)).z);
//				Eigen::Vector3f error_i=pt-Rrand*ipt-Trand;
//				float distz=FeaturePointCloud[0].points[correspondences[u]].z;
//				float acc=.3;//pow(distz,2)*0.0075*sqrt(3);
//				//std::cout<<"distz:"<<distz<<"acc ohne sqrt="<<acc<<"acc mit sqrt="<<acc*sqrt(3)<<std::endl;
//				if(abs(error_i.norm())<=acc)//acc*sqrt(3))
//				{
//					inl_tmp++;
//					correspondences_inliersKey_tmp.push_back(corr_Keyframe.at(u));
//					correspondences_inliersSBA_tmp.push_back(corr_sba.at(u));
//					err_ges_temp+=abs(error_i.norm());
//				}
//				else
//				{
//					err_ges_temp+=abs(error_i.norm());//acc*sqrt(3);
//					//							correspondences_outliers.push_back(corr_)
//				}
//			}
//
//		}
//
//		if(err_ges_temp<err_ges)
//		{
//			//			corresKey.clear();
//			//			corresSBA.clear();
//			//			//		corr_Keyframe_inliers.resize (inliers.size ());
//			//			// Copy just the inliers
//			//			for (size_t i = 0; i < inliers.size (); ++i)
//			//			{
//			//				corresKey.push_back(corr_Keyframe[inliers[i]]);
//			//				corresSBA.push_back(corr_sba[inliers[i]]);
//			err_ges=err_ges_temp;
//			//			std::cout<<"err_ges"
//			corresKey=correspondences_inliersKey_tmp;
//			corresSBA=correspondences_inliersSBA_tmp;
//			//					correspondences_inliers=correspondences_inliers_tmp;
//		}
//
//
//
//
//
//
//
//		//		//		std::cout<<"ransac tfm:"<<std::endl<<tfm<<std::endl<<"inl="<<inl<<std::endl;
//		//		//				std::cout<<"T="<<std::endl<<T<<std::endl;
//		//		//				std::cout<<"R="<<std::endl<<R<<std::endl;
//		//
//		//
//		//		if (inl > ransac_inliers)
//		//		{
//		//			//			std::cout<<"ransac fertig tfm:"<<std::endl<<tfm<<std::endl<<"inl="<<inl<<std::endl;
//		//
//		//			posest_transformation=Eigen::Matrix4f::Identity();
//		//			posest_transformation.block<3,3>(0,0)=R;
//		//			posest_transformation.block<3,1>(0,3)=T;
//		//			std::cout<<"anzahl inliers: "<<inl<<std::endl;
//		//			std::cout<<"Final transformation:"<<std::endl<<posest_transformation<<std::endl;
//		//
//		//
//		//			return;
//		//		}
//		//		if(inl<ransac_inliers&&i>ransac_it-1)
//		//		{
//		//			correspondences_inliers=correspondences_inliers_tmp;
//		//			posest_transformation=trans_tmp;
//		//			std::cout<<"anzahl inliers: "<<inl<<std::endl;
//		//
//		//			std::cout<<"No convergence: Final transformation:"<<std::endl<<posest_transformation<<std::endl;
//		//			return;
//		//
//		//
//		//		}
//	}
//	std::cout<<"size of pointcloud of keyframe"<<Data.Points.size()<<std::endl;
//	std::cout<<"size of matches of keyframe:"<<matches.size()<<std::endl;
//
//	//		//RANSAC
//	//
//	//
//	//		typedef pcl::SampleConsensusModelRegistration<Point>::Ptr SampleConsensusModelRegistrationPtr;
//	//
//	//		SampleConsensusModelRegistrationPtr model;
//	//
//	//		PointCloud tmp;
//	//		tmp.header.frame_id="/pgraph";
//	//		pcl::transformPointCloud(Data.Points,tmp,Data.Transformation);
//	//
//	//		model.reset (new pcl::SampleConsensusModelRegistration<Point> (tmp.makeShared (), corr_Keyframe));
//	//
//	//
//	//		model->setInputTarget (tmpsbacloud.makeShared(), corr_sba);
//	//		pcl::RandomSampleConsensus<Point> sac (model, .2);
//	//		sac.setMaxIterations (ransac_it);
//	//
//	//
//	//		// Compute the set of inliers
//	//
//	//		if (!sac.computeModel (5))
//	//		{
//	//			corr_Keyframe_inliers = corr_Keyframe;
//	//		}
//	//		else
//	//		{
//	//			std::vector<int> inliers;
//	//			// Get the inliers
//	//			sac.getInliers (inliers);
//	//			corresKey.clear();
//	//			corresSBA.clear();
//	//			//		corr_Keyframe_inliers.resize (inliers.size ());
//	//			// Copy just the inliers
//	//			for (size_t i = 0; i < inliers.size (); ++i)
//	//			{
//	//				corresKey.push_back(corr_Keyframe[inliers[i]]);
//	//				corresSBA.push_back(corr_sba[inliers[i]]);
//	//			}
//	//
//	//		}
//	std::cout<<"size of inliers of keyframe:"<<corresKey.size()<<std::endl;
//
//
//
//
//
//}

//////code partially copied from posest package (vslam) and from paper Arun, Huang, Blostein: Least-Squares Fitting of Two 3D Point Sets
void EXTRACT::PosEst()
{
	correspondences_outliers.clear();

	Eigen::Matrix4f frametrans=Eigen::Matrix4f::Identity();

	Eigen::Vector3f translation;
	float betrag;




	Eigen::Matrix4f ransacTrans;

	Eigen::Matrix4f trans_tmp;
	std::vector<int> correspondences_inliers_tmp;
	int nmatch=correspondences.size();
	std::cout<<"nmatch="<<nmatch<<std::endl;
	//ransac_inliers=int(.1*nmatch);
	double err_ges_temp=0;
	double err_ges=100000;
	if(nmatch<min_inliers)
	{
		//ransac_inliers=3;
		std::cout<<"to few correspondences"<<std::endl;
		return;
	}

	for(uint i=0;i<ransac_it;i++)
	{
		Eigen::Matrix4f randomTrans;
		err_ges_temp=0;
		int inl_tmp=0;
		// find a candidate
		int a=rand()%nmatch;
		int b = a;
		while (a==b)
			b=rand()%nmatch;
		int c = a;
		while (a==c || b==c)
			c=rand()%nmatch;

		std::vector<int> rand_corr;
		rand_corr.push_back(0);
		rand_corr.push_back(1);
		rand_corr.push_back(2);
		PointCloud rand_PointCloud[2];
		rand_PointCloud[0].push_back(FeaturePointCloud[0].points.at(a));
		rand_PointCloud[0].push_back(FeaturePointCloud[0].points.at(b));
		rand_PointCloud[0].push_back(FeaturePointCloud[0].points.at(c));
		rand_PointCloud[1].push_back(FeaturePointCloud[1].points.at(a));
		rand_PointCloud[1].push_back(FeaturePointCloud[1].points.at(b));
		rand_PointCloud[1].push_back(FeaturePointCloud[1].points.at(c));

		pcl::estimateRigidTransformationSVD(rand_PointCloud[1],rand_corr,rand_PointCloud[0],rand_corr, randomTrans);



		//		Eigen::Vector3f p1(FeaturePointCloud[0].points[correspondences[a]].x,FeaturePointCloud[0].points[correspondences[a]].y,FeaturePointCloud[0].points[correspondences[a]].z),
		//				p2(FeaturePointCloud[0].points[correspondences[b]].x,FeaturePointCloud[0].points[correspondences[b]].y,FeaturePointCloud[0].points[correspondences[b]].z),
		//				p3(FeaturePointCloud[0].points[correspondences[c]].x,FeaturePointCloud[0].points[correspondences[c]].y,FeaturePointCloud[0].points[correspondences[c]].z),
		//				p1_corr(FeaturePointCloud[1].points[correspondences[a]].x,FeaturePointCloud[1].points[correspondences[a]].y,FeaturePointCloud[1].points[correspondences[a]].z),
		//				p2_corr(FeaturePointCloud[1].points[correspondences[b]].x,FeaturePointCloud[1].points[correspondences[b]].y,FeaturePointCloud[1].points[correspondences[b]].z),
		//				p3_corr(FeaturePointCloud[1].points[correspondences[c]].x,FeaturePointCloud[1].points[correspondences[c]].y,FeaturePointCloud[1].points[correspondences[c]].z);

		//		PointCloud p, p_corr;
		//		pcl::PointXYZRGB p1, p2, p3, p1_corr, p2_corr, p3_co
		//
		//		p.push_back();

		//		Eigen::Vector3f p=1/3.0*(p1+p2+p3);
		//		Eigen::Vector3f p_corr=1/3.0*(p1_corr+p2_corr+p3_corr);
		//
		//		Eigen::Vector3f q1=p1-p;
		//		Eigen::Vector3f q2=p2-p;
		//		Eigen::Vector3f q3=p3-p;
		//
		//		Eigen::Vector3f q1_corr=p1_corr-p_corr;
		//		Eigen::Vector3f q2_corr=p2_corr-p_corr;
		//		Eigen::Vector3f q3_corr=p3_corr-p_corr;
		//
		//		Eigen::Matrix3f H=q1*q1_corr.transpose()+q2*q2_corr.transpose()+q3*q3_corr.transpose();
		//
		//		Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
		//		Eigen::Matrix3f V = svd.matrixV();
		//		Eigen::Matrix3f R=V * svd.matrixU().transpose();
		//
		//		double det = R.determinant();
		//		if (det < 0.0)
		//		{
		//			//			std::cout<<"det smaller than 0"<<std::endl;
		//			//nneg++;
		//			V.col(2) = V.col(2)*-1.0;
		//			R = V * svd.matrixU().transpose();
		//		}
		//		Eigen::Vector3f T=p_corr-R*p;
		//		Eigen::Matrix<float,3,4> tfmpe;
		//
		//		tfmpe.block<3,3>(0,0) = R;
		//		tfmpe.col(3) = T;

		Eigen::Vector3f Trand=randomTrans.block<3,1>(0,3);
		Eigen::Matrix3f Rrand=randomTrans.block<3,3>(0,0);
		//		correspondences_inliers.clear();
		correspondences_inliers_tmp.clear();
		// find inliers, based on image reprojection
		for (int u=0; u<nmatch; u++)
		{
			if(u!=a&&u!=b&&u!=c)
			{
				Eigen::Vector3f pt(FeaturePointCloud[0].points[correspondences[u]].x,FeaturePointCloud[0].points[correspondences[u]].y,FeaturePointCloud[0].points[correspondences[u]].z);
				Eigen::Vector3f ipt(FeaturePointCloud[1].points[correspondences[u]].x,FeaturePointCloud[1].points[correspondences[u]].y,FeaturePointCloud[1].points[correspondences[u]].z);
				Eigen::Vector3f error_i=pt-Rrand*ipt-Trand;
				float distz=FeaturePointCloud[0].points[correspondences[u]].z;
				float acc=pow(distz,2)*0.0075*sqrt(3);
				//std::cout<<"distz:"<<distz<<"acc ohne sqrt="<<acc<<"acc mit sqrt="<<acc*sqrt(3)<<std::endl;
				if(abs(error_i.norm())<=acc)//acc*sqrt(3))
				{
					inl_tmp++;
					correspondences_inliers_tmp.push_back(correspondences[u]);
					err_ges_temp+=abs(error_i.norm());
				}
				else
				{
					err_ges_temp+=acc;//acc*sqrt(3);
					correspondences_outliers.push_back(correspondences[u]);

				}
			}

		}

		if(err_ges_temp<err_ges)
		{
			err_ges=err_ges_temp;
			//			std::cout<<"err_ges"
			correspondences_inliers=correspondences_inliers_tmp;
		}







		//		//		std::cout<<"ransac tfm:"<<std::endl<<tfm<<std::endl<<"inl="<<inl<<std::endl;
		//		//				std::cout<<"T="<<std::endl<<T<<std::endl;
		//		//				std::cout<<"R="<<std::endl<<R<<std::endl;
		//
		//
		//		if (inl > ransac_inliers)
		//		{
		//			//			std::cout<<"ransac fertig tfm:"<<std::endl<<tfm<<std::endl<<"inl="<<inl<<std::endl;
		//
		//			posest_transformation=Eigen::Matrix4f::Identity();
		//			posest_transformation.block<3,3>(0,0)=R;
		//			posest_transformation.block<3,1>(0,3)=T;
		//			std::cout<<"anzahl inliers: "<<inl<<std::endl;
		//			std::cout<<"Final transformation:"<<std::endl<<posest_transformation<<std::endl;
		//
		//
		//			return;
		//		}
		//		if(inl<ransac_inliers&&i>ransac_it-1)
		//		{
		//			correspondences_inliers=correspondences_inliers_tmp;
		//			posest_transformation=trans_tmp;
		//			std::cout<<"anzahl inliers: "<<inl<<std::endl;
		//
		//			std::cout<<"No convergence: Final transformation:"<<std::endl<<posest_transformation<<std::endl;
		//			return;
		//
		//
		//		}
	}


	//post ransac computations:

	if(showDisplay)
	{
		ROS_INFO("number of correspondences  after ransac %d",correspondences_source_good.size());
		averageNumberOfCorrespondences+=correspondences_inliers.size();
		ROS_INFO("average correspondences=%f",averageNumberOfCorrespondences/float(compute_counter));
	}
	ransac_inliers=correspondences_inliers.size();

	//	std::cout<<"minransac keyfram"<<next_keyframe_inlier<<std::endl;
	//	std::cout<<"next keyframe:"<<next_keyframe<<std::endl;
	//Check whether we need next keyframe
	if(ransac_inliers<next_keyframe_inlier)
		next_keyframe=true;
	else
		next_keyframe=false;

	// Check whether we have enough correspondences

	if (ransac_inliers < min_inliers)
	{
		ROS_ERROR ("[pcl::::computeTransformation] Not enough correspondences found. Relax your threshold parameters.");
		compute_transform_success=false;
		countOfBadCorrespondences++;
	}
	else
		compute_transform_success=true;




	//Berechnung der Transformation
	if(compute_transform_success)
	{
		if(transformation_at_least_once_computed)
			transformation_at_least_twice_computed=true;
		transformation_at_least_once_computed=true;

		previous_transformation=transformation_;
		pcl::estimateRigidTransformationSVD(FeaturePointCloud[1],correspondences_inliers,FeaturePointCloud[0],correspondences_inliers, transformation_);
		if(transformation_at_least_twice_computed)
		{
			frametrans=KeyframeDataVector.at(actual_keyframe).Transformation*transformation_*previous_transformation.inverse()*transformation_;
			translation=frametrans.block<3,1>(0,3);
			betrag=sqrt(translation[0]*translation[0]+translation[1]*translation[1]+translation[2]*translation[2]);
			if(betrag>1)
			{
				transformation_=Eigen::Matrix4f::Identity();
				ROS_ERROR ("TRANSLATION larger than 1 meter");

			}
		}
		transformOld=KeyframeDataVector.at(actual_keyframe).Transformation*transformation_;



	}
	else
	{

		if(transformation_at_least_twice_computed)
		{
			frametrans=KeyframeDataVector.at(actual_keyframe).Transformation*transformation_*previous_transformation.inverse()*transformation_;
			translation=frametrans.block<3,1>(0,3);
			betrag=sqrt(translation[0]*translation[0]+translation[1]*translation[1]+translation[2]*translation[2]);
			//				if(betrag>1)
			//				{
			//					frametrans=Eigen::Matrix4f::Identity();
			//					ROS_ERROR ("TRANSLATION to large");
			//
			//				}
			transformOld=frametrans;
		}
		else
			transformOld=Eigen::Matrix4f::Identity();

	}

	//	if(ransac_inliers<next_keyframe_inlier)
	//					next_keyframe=true;
	//				else
	//					next_keyframe=false;

	// Check whether we have enough correspondences

	//				if (correspondences_inliers.size() < min_inliers)
	//				{
	//					ROS_ERROR ("[pcl::::computeTransformation] Not enough correspondences found in posest. Relax your threshold parameters.");
	////					compute_transform_success=false;
	////					countOfBadCorrespondences++;
	//				}
	//
	//
	//				//Berechnung der Transformation
	//				if(compute_transform_success)
	//				{
	//
	//					pcl::estimateRigidTransformationSVD(FeaturePointCloud[1],correspondences_inliers,FeaturePointCloud[0],correspondences_inliers, ransacTrans);
	//					transformRANSACOwn=keyTrans*ransacTrans;
	//
	//
	//				}
	//				else
	//				{
	//
	//					if(transformation_at_least_twice_computed)
	//					{
	//						frametrans=keyTrans*transformation_*previous_transformation.inverse()*transformation_;
	//						translation=frametrans.block<3,1>(0,3);
	//						betrag=sqrt(translation[0]*translation[0]+translation[1]*translation[1]+translation[2]*translation[2]);
	//						if(betrag>1)
	//						{
	//							frametrans=Eigen::Matrix4f::Identity();
	//							ROS_ERROR ("TRANSLATION to large");
	//
	//						}
	//						transformOld=frametrans;
	//					}
	//					else
	//						transformOld=Eigen::Matrix4f::Identity();
	//
	//				}
	//				if(next_keyframe)
	//				{
	//					keyTrans=transformOld;
	//
	//				}
	//	averageNumberOfCorrespondences_posest+=correspondences_inliers.size();
	//	ROS_INFO("average correspondences=%f",averageNumberOfCorrespondences_posest/float(compute_counter));
	//	std::cout<<"transformransacown_"<<std::endl<<transformRANSACOwn<<std::endl;
	correspondences_source_good=correspondences_inliers;
}

void EXTRACT::createCorrespondingPointcloud(struct FrameData& Data0,PointCloud& Cloud0,struct FrameData& Data1,PointCloud& Cloud1,std::vector<int>& correspondvector,vector<cv::DMatch>& matches,bool show,std::vector<cv::KeyPoint>& kpts0,std::vector<cv::KeyPoint>& kpts1)

{

	std::vector<int> corr_tmp;
	std::vector<int> corr_matches_tmp;

	PointCloud FeatureTmp[2];
	FeatureTmp[0].header.frame_id="/pgraph";
	FeatureTmp[1].header.frame_id="/pgraph";



	Point push_back_point;

	{
		ROS_WARN("in cvline matches");


		for (size_t i = 0; i < matches_popcount.size(); i++)
		{
			FeatureTmp[0].push_back(Data0.Points.points[matches_popcount[i].queryIdx]);
			corr_tmp.push_back(i);
			if(show)
			{
				cvLine( imgadd, cvPoint(kpts0[matches_popcount[i].queryIdx].pt.x, kpts0[matches_popcount[i].queryIdx].pt.y),
						cvPoint(kpts1[matches_popcount[i].trainIdx].pt.x, 480+kpts1[matches_popcount[i].trainIdx].pt.y), colors[7] );

			}
			FeatureTmp[1].push_back(Data1.Points.points[matches_popcount[i].trainIdx]);


		}

	}

	Cloud0=FeatureTmp[0];
	Cloud1=FeatureTmp[1];
	correspondvector=corr_tmp;
}

void EXTRACT::createCorrespondingPointcloud(struct FrameData& Data0,PointCloud& Cloud0,struct FrameData& Data1,PointCloud& Cloud1,std::vector<int>& correspondvector,vector<cv::DMatch>& matches)

{

	std::vector<int> corr_tmp;
	std::vector<int> corr_matches_tmp;

	PointCloud FeatureTmp[2];
	FeatureTmp[0].header.frame_id="/pgraph";
	FeatureTmp[1].header.frame_id="/pgraph";



	Point push_back_point;

	{

		for (size_t i = 0; i < matches_popcount.size(); i++)
		{
			FeatureTmp[0].push_back(Data0.Points.points[matches_popcount[i].queryIdx]);
			corr_tmp.push_back(i);
			FeatureTmp[1].push_back(Data1.Points.points[matches_popcount[i].trainIdx]);


		}

	}

	Cloud0=FeatureTmp[0];
	Cloud1=FeatureTmp[1];
	correspondvector=corr_tmp;
}

//void EXTRACT::doSBAthing(std::vector<int> correspond,struct FrameData data0, struct FrameData data1,vector<cv::DMatch> matches)
//{
//	//	//Publish Camera poses for SBA
//	//			camera_pos.pose.orientation.x=quat_rot.x()/quat_rot.w();
//	//			camera_pos.pose.orientation.y=quat_rot.y()/quat_rot.w();
//	//			camera_pos.pose.orientation.z=quat_rot.z()/quat_rot.w();
//	//			camera_pos.header.frame_id="/pgraph";
//	//			ros::Time tstamp=ros::Time::now();
//	//			camera_pos.header.stamp=tstamp;
//	//			camera_pos.type=visualization_msgs::Marker::CUBE;
//	//			camera_pos.scale.x=1;
//	//			camera_pos.scale.y=1;
//	//			camera_pos.scale.z=1;
//	//			camera_pos.pose.position.x=trans_vec[0];
//	//			camera_pos.pose.position.y=trans_vec[1];
//	//			camera_pos.pose.position.z=trans_vec[2];
//
//	sba::SysSBA sys;
//
//
//	Eigen::Quaternion<float> quat_test(data0.Transformation.block<3,3>(0,0));
//
//	Eigen::Vector4d trans((double)data0.Transformation.block<3,1>(0,3)[0], (double)data0.Transformation.block<3,1>(0,3)[1], (double)data0.Transformation.block<3,1>(0,3)[2], (double)1);
//
//	Eigen::Quaterniond rot((double)quat_test.w(),(double)quat_test.x(),(double)quat_test.y(),(double)quat_test.z());
//	rot.normalize();
//
//	sys.addNode(trans, rot, cam_params, false);
//
//	Eigen::Quaternion<float> quat_test2(data1.Transformation.block<3,3>(0,0));
//
//	Eigen::Vector4d trans2((double)data1.Transformation.block<3,1>(0,3)[0], (double)data1.Transformation.block<3,1>(0,3)[1], (double)data1.Transformation.block<3,1>(0,3)[2], (double)1);
//
//	Eigen::Quaterniond rot2((double)quat_test2.w(),(double)quat_test2.x(),(double)quat_test2.y(),(double)quat_test2.z());
//	rot2.normalize();
//
//	std::cout<<"quaternion x:"<<quat_test2.x()<<"y:"<<quat_test2.y()<<"z:"<<quat_test2.z()<<"w:"<<quat_test2.w()<<std::endl;
//	std::cout<<"quaternion xdouble:"<<(double)quat_test2.x()<<"y:"<<(double)quat_test2.y()<<"z:"<<(double)quat_test2.z()<<"w:"<<(double)quat_test2.w()<<std::endl;
//	std::cout<<"quaternion rot:"<<rot2.x()<<"y:"<<rot2.y()<<"z:"<<(double)rot2.z()<<"w:"<<rot2.w()<<std::endl;
//
//	sys.addNode(trans2, rot2, cam_params, false);
//
//	for(uint i=0;i<correspond.size();i++)
//	{
//		Eigen::Vector4d temppoint((double)data0.Points.points[correspond.at(i)].x, (double)data0.Points.points[correspond.at(i)].y, (double)data0.Points.points[correspond.at(i)].z, (double)1);
//		points.push_back(temppoint);
//		project_counter++;
//	}
//
//	for(uint i=0;i<correspond.size();i++)
//	{
//		Eigen::Vector4d temppoint((double)data1.Points.points[correspond.at(i)].x, (double)data1.Points.points[correspond.at(i)].y, (double)data1.Points.points[correspond.at(i)].z, (double)1);
//		sys.addPoint(temppoint);
//		points.push_back(temppoint);
//		project_counter++;
//	}
//	// Project points into nodes.
//	for (uint i = 0; i < correspond.size(); i++)
//	{
//
//		Vector2d proj0(data0.Keypoints[matches[correspond.at(i)].queryIdx].pt.x,data0.Keypoints[matches[correspond.at(i)].queryIdx].pt.y);
//		sys.addMonoProj(0,i,proj0);
//		Vector2d proj1(data1.Keypoints[matches[correspond.at(i)].trainIdx].pt.x,data1.Keypoints[matches[correspond.at(i)].trainIdx].pt.y);
//		sys.addMonoProj(1,i,proj1);
//
//	}
//
//
//	sys.doSBA(5);
//
//	std::cout<<"new trans:"<<std::endl<<sys.nodes[1].trans<<std::endl;
//	std::cout<<"new quaternion x:"<<sys.nodes[1].qrot.x()<<"y:"<<sys.nodes[1].qrot.y()<<"z:"<<sys.nodes[1].qrot.z()<<"w:"<<sys.nodes[1].qrot.w()<<std::endl;
//
//	std::cout<<"old trans:"<<std::endl<<sys.nodes[1].oldtrans<<std::endl;
//	std::cout<<"old quaternion x:"<<sys.nodes[1].oldqrot.x()<<"y:"<<sys.nodes[1].oldqrot.y()<<"z:"<<sys.nodes[1].oldqrot.z()<<"w:"<<sys.nodes[1].oldqrot.w()<<std::endl;
//
//}
//
//void EXTRACT::doSBAwithMap()
//{
//	//	for()
//
//	//	vector<cv::DMatch> matches;
//	sba::SysSBA sys;
//	//	std::vector<int> corres_key;
//	//	std::vector<int> corres_sba;
//	//
//	std::cout<<"keyframedatavector.size:"<<KeyframeDataVector.size();
//	for(uint i=0;i<KeyframeDataVector.size();i++)
//	{
//		Eigen::Quaternion<float> quat_test(KeyframeDataVector.at(i).Transformation.block<3,3>(0,0));
//
//		Eigen::Vector4d trans((double)KeyframeDataVector.at(i).Transformation.block<3,1>(0,3)[0], (double)KeyframeDataVector.at(i).Transformation.block<3,1>(0,3)[1], (double)KeyframeDataVector.at(i).Transformation.block<3,1>(0,3)[2], (double)1);
//
//		Eigen::Quaterniond rot((double)quat_test.w(),(double)quat_test.x(),(double)quat_test.y(),(double)quat_test.z());
//		rot.normalize();
//
//		if(i==2)
//		{
//			std::cout<<"before trans:"<<std::endl<<trans<<std::endl;
//			std::cout<<"before quaternion x:"<<rot.x()<<"y:"<<rot.y()<<"z:"<<rot.z()<<"w:"<<rot.w()<<std::endl;
//
//		}
//		if(i==0)
//		sys.addNode(trans, rot, cam_params, true);
//		else
//		sys.addNode(trans, rot, cam_params, false);
//
//		IplImage *tmp=cvCreateImage( cvSize(640, 480), IPL_DEPTH_8U, 3 );
//		track_vector.push_back(tmp);
//	}
//	std::cout<<"sys.node.size"<<sys.nodes.size();
//
//	//
//	//	std::cout<<"nach addnodes"<<std::endl;
//	//	//		Eigen::Quaternion<float> quat_test2(data1.Transformation.block<3,3>(0,0));
//	//	//
//	//	//		Eigen::Vector4d trans2((double)data1.Transformation.block<3,1>(0,3)[0], (double)data1.Transformation.block<3,1>(0,3)[1], (double)data1.Transformation.block<3,1>(0,3)[2], (double)1);
//	//	//
//	//	//		Eigen::Quaterniond rot2((double)quat_test2.w(),(double)quat_test2.x(),(double)quat_test2.y(),(double)quat_test2.z());
//	//	//		rot2.normalize();
//	//	//
//	//	//		std::cout<<"quaternion x:"<<quat_test2.x()<<"y:"<<quat_test2.y()<<"z:"<<quat_test2.z()<<"w:"<<quat_test2.w()<<std::endl;
//	//	//		std::cout<<"quaternion xdouble:"<<(double)quat_test2.x()<<"y:"<<(double)quat_test2.y()<<"z:"<<(double)quat_test2.z()<<"w:"<<(double)quat_test2.w()<<std::endl;
//	//	//		std::cout<<"quaternion rot:"<<rot2.x()<<"y:"<<rot2.y()<<"z:"<<(double)rot2.z()<<"w:"<<rot2.w()<<std::endl;
//	//	//
//	//	//		sys.addNode(trans2, rot2, cam_params, false);
//	//
//	//	//		for(uint i=0;i<correspond.size();i++)
//	//	//		{
//	//	//			Eigen::Vector4d temppoint((double)data0.Points.points[correspond.at(i)].x, (double)data0.Points.points[correspond.at(i)].y, (double)data0.Points.points[correspond.at(i)].z, (double)1);
//	//	//			points.push_back(temppoint);
//	//	//			project_counter++;
//	//	//		}
//	//
//
//	int point_counter=0;
//	std::cout<<"before sbapoint"<<std::endl;
//	for(uint i=0;i<SBAPoint.size();i++)
//	{
//		//		std::cout<<"i:"<<i<<std::endl;
//		if(SBAPoint.at(i).cameras.size()>2)
//		{
//			//			std::cout<<"SBAPoint.at(i).cameras.size()"<<std::endl;
//			//			std::cout<<SBAPoint.at(i).cameras.size()<<std::endl;
//			Eigen::Vector4d temppoint((double)SBAPoint.at(i).PointXYZ.x, (double)SBAPoint.at(i).PointXYZ.y, (double)SBAPoint.at(i).PointXYZ.z, (double)1);
//			//		rgbstuff
//			//		std::cout<<"1"<<std::endl;
//			STORE_POINTS_IN_SBA
//			//		std::cout<<"2"<<std::endl;
//
//			for(uint k=0;k<SBAPoint.at(i).cameras.size();k++)
//			{
//				//			std::cout<<"3"<<std::endl;
//
//				Vector2d proj0(SBAPoint.at(i).cameras.at(k).at(1),SBAPoint.at(i).cameras.at(k).at(2));
//				//				std::cout<<"matches.size()"<<matches.size()<<std::endl;
//				//				std::cout<<"corres_sba.at(k)"<<corres_sba.at(k)<<std::endl;
//				//			std::cout<<"4"<<std::endl;
//				//			std::cout<<"params, SBAPoint.at(i).cameras.at(k).at(0):"<<SBAPoint.at(i).cameras.at(k).at(0)<<"i:"<<i<<"projx:"<<proj0[0]<<"projy:"<<proj0[1]<<std::endl;
//
//				if(SBAPoint.at(i).cameras.at(k).at(0)==keyframeforreprojection)
//				{
////					circle.x=SBAPoint.at(i).cameras.at(k).at(1);
////					circle.y=SBAPoint.at(i).cameras.at(k).at(2);
//					pointsWithProjectionTo2.push_back(point_counter);
////					cvCircle(track_reprojection_error,circle,3,colors[5],1,8,0);
//					//					circle.x=kpts[k][i].pt.x;
//					//					circle.y=kpts[k][i].pt.y;
//					//					cvCircle( cv_image[k], circle, 1, colors[5], 1, 8, 0 );
//
//				}
//				sys.addMonoProj(SBAPoint.at(i).cameras.at(k).at(0),point_counter,proj0);
//				std::cout<<"SBAPoint.at(i).cameras.at(k)"<<SBAPoint.at(i).cameras.at(k).at(0)<<std::endl;
//				//			STORE_POINTS_IN_SBA
//				//			std::cout<<"5"<<std::endl;
//			}
//			point_counter++;
//
//		}
//
//	}
//
//	for(uint i=0;i<sys.tracks.size();i++)
//	{
////		ProjMap::iterator it=sys.tracks.at(i).projections.begin();
//
//		uint size_of_tracks=sys.tracks.at(i).projections.size();
//		std::cout<<"sizeoftracks"<<size_of_tracks<<std::endl;
//		for(ProjMap::iterator it=sys.tracks.at(i).projections.begin();it!=sys.tracks.at(i).projections.end();++it)
//		{
//			std::cout<<"sys.tracks.at(i).projection.size()"<<sys.tracks.at(i).projections.size()<<std::endl;
//			std::cout<<"i:"<<i<<std::endl;
////			std::cout<<"d:"<<d<<std::endl;
//			std::cout<<"sys.tracks.size()"<<sys.tracks.size()<<std::endl;
//
//			std::cout<<".5"<<std::endl;
//			int actual_node=it->first;
//
//			std::cout<<"actual_node:"<<actual_node<<std::endl;
//			circle.x=sys.tracks.at(i).projections.at(actual_node).kp[0];
//			circle.y=sys.tracks.at(i).projections.at(actual_node).kp[1];
//			std::cout<<"1"<<std::endl;
//			cvCircle(track_vector.at(actual_node),circle,4,colors[2],1,8,0);
//			std::cout<<"2"<<std::endl;
//
//			Vector2d proj;
//
//			// Project the point into the node's image coordinate system.
//			sys.nodes[actual_node].setProjection();
//			sys.nodes[actual_node].project2im(proj, sys.tracks.at(i).point);
//			std::cout<<"3"<<std::endl;
//
//			//		if(i<4)
//			//					std::cout<<"the point pos is:"<<sys.tracks.at(pointsWithProjectionTo2.at(i)).point<<std::endl;
//			//		std::cout<<"sys.tracks.at(i).projections.at(2)[1]"<<sys.tracks.at(pointsWithProjectionTo2.at(i)).projections.at(2).ndi<<std::endl;
//			circle.x=proj[0];
//			circle.y=proj[1];
//			cvCircle(track_vector.at(actual_node),circle,1,colors[4],1,8,0);
//			std::cout<<"4"<<std::endl;
//
//		}
//	}
//
//	for(uint i=0;i<track_vector.size();i++)
//	{
////		char name[10];
////		sprintf(name, "pic%d", i);
//////		name[3]=i;
////		std::cout<<"saving image:"<<name<<std::endl;
////		const char *nameini;
////		sprintf(*nameini, "pic%d", i);
//		std::stringstream sstr;
//		sstr << i;
//		std::string str1 = sstr.str();//std::string str2 = boost::lexical_cast<std::string>(my_int);
////		char name=(char)str1.c_str();
////		const char* name2=name;
////		show all images
//
////		cvShowImage(str1.c_str(),track_vector.at(i));
//		cvWaitKey(30);
//	}
//	for(uint f=0; f<track_vector.size();f++)
//		cvReleaseImage(&track_vector.at(f));
//
//	track_vector.clear();
//	for(uint i=0;i<KeyframeDataVector.size();i++)
//		{
//			IplImage *tmp=cvCreateImage( cvSize(640, 480), IPL_DEPTH_8U, 3 );
//			track_vector.push_back(tmp);
//		}
//	//	int node_counter=0;
//	//
//	//	for(uint i=0;i<KeyframeDataVector.size();i++)
//	//	{
//	//		PointCloud sbacloud;
//	//
//	//		std::cout<<"ransac"<<std::endl;
//	//		//			SBARANSAC(KeyframeDataVector.at(i),matches,corres_key,corres_sba,i,sbacloud);
//	//		ownSBARANSAC(KeyframeDataVector.at(i),matches,corres_key,corres_sba,i,sbacloud);
//	//		std::cout<<"ransac"<<std::endl;
//	//		std::cout<<"corres_sba.size()"<<corres_sba.size()<<std::endl;
//	//		PointCloud seenPoints;
//	//		seenPoints.header.frame_id="/pgraph";
//	//
//	//		int size=0;
//	//		for(uint d=0;d<SBAmap.Points.size();d++)
//	//		{
//	//			std::cout<<"size of points:"<<SBAmap.Points.at(d).size()<<"size of desc:"<<SBAmap.Descriptor.at(d).rows<<std::endl;
//	//			size+=SBAmap.Points.at(d).size();
//	//		}
//	//
//	//		std::cout<<"size"<<size<<std::endl;
//	//
//	//		int last_start;
//	//		int end_number;
//	//		int start_number;
//	//
//	//		if(i==0)
//	//		{
//	//			start_number=0;
//	//			end_number=SBAmap.Points.at(0).size();
//	//			last_start=end_number;
//	//		}
//	//		else
//	//		{
//	//			start_number=last_start;
//	//			end_number=start_number+SBAmap.Points.at(i).size();
//	//			last_start=start_number;
//	//
//	//		}
//	//
//	//		std::cout<<"numbers should be between"<<start_number<<"and"<<end_number<<std::endl;
//	//
//	//
//	//
//	//
//	//		for(uint w=0;w<corres_sba.size();w++)
//	//		{
//	//			seenPoints.points.push_back(sbacloud.points.at(corres_sba.at(w)));
//	//			//				std::cout<<"corres_sba.at(w)"<<corres_sba.at(w)<<std::endl;
//	//		}
//	//
//	//		std::cout<<"seenPoints.size()"<<seenPoints.size()<<std::endl;
//	//		ROS_INFO("Sleeping for .5 seconds to show seen points by camera %d.",i);
//	////		SeenKeyFramePoints.publish(seenPoints);
//	//		ros::Duration(0.5).sleep();
//	//
//	//
//	//
//	//		//		std::cout<<"corres_key.size()"<<corres_key.size()<<std::endl;
//	//		// Project points into nodes.
//	//		PointCloud tmp;
//	//		pcl::transformPointCloud(KeyframeDataVector.at(i).Points,tmp,KeyframeDataVector.at(i).Transformation);
//	//		Eigen::Matrix4f sbatrans;
//	//		std::cout<<"before svd"<<std::endl;
//	//		PointCloud tmp2;
//	//
//	//		afterSBAPointCloud.header.frame_id="/pgraph";
//	//		tmp2.header.frame_id="/pgraph";
//	//
//	//		pcl::estimateRigidTransformationSVD(KeyframeDataVector.at(i).Points,corres_key,sbacloud,corres_sba,sbatrans);
//	//		if(corres_key.size()>70)
//	//		{
//	//			std::cout<<"copypointclod"<<std::endl;
//	//			tmp2.header.frame_id="/pgraph";
//	//
//	//
//	//			pcl::transformPointCloud(KeyframeDataVector.at(i).KinectCloud,tmp2,sbatrans);
//	//
//	//			afterSBAPointCloud+=tmp2;
//	//			Eigen::Quaternion<float> quat_test(KeyframeDataVector.at(i).Transformation.block<3,3>(0,0));
//	//
//	//			Eigen::Vector4d trans((double)KeyframeDataVector.at(i).Transformation.block<3,1>(0,3)[0], (double)KeyframeDataVector.at(i).Transformation.block<3,1>(0,3)[1], (double)KeyframeDataVector.at(i).Transformation.block<3,1>(0,3)[2], (double)1);
//	//
//	//			Eigen::Quaterniond rot((double)quat_test.w(),(double)quat_test.x(),(double)quat_test.y(),(double)quat_test.z());
//	//			rot.normalize();
//	//
//	//			if(i==2)
//	//			{
//	//				std::cout<<"before trans:"<<std::endl<<trans<<std::endl;
//	//				std::cout<<"before quaternion x:"<<rot.x()<<"y:"<<rot.y()<<"z:"<<rot.z()<<"w:"<<rot.w()<<std::endl;
//	//
//	//			}
//	//			sys.addNode(trans, rot, cam_params, false);
//	//			//			{
//	//			//		for(uint k=0;k<SBAmap.Points.at(i).size();k++)
//	//			//		{
//	//			//			Eigen::Vector4d temppoint((double)SBAmap.Points.at(i).points[k].x, (double)SBAmap.Points.at(i).points[k].y, (double)SBAmap.Points.at(i).points[k].z, (double)1);
//	//			//	//		rgbstuff
//	//			////			sys.addPoint(temppoint);
//	//			//			STORE_POINTS_IN_SBA
//	//			//		}
//	//			//			}
//	//
//	//
//	//
//	//			std::cout<<"sbatrans"<<sbatrans<<std::endl;;
//	//			for (uint k = 0; k < corres_key.size(); k++)
//	//			{
//	//				//				std::cout<<"KeyframeDataVector.at(i).Keypoints.size();"<<KeyframeDataVector.at(i).Keypoints.size()<<std::endl;
//	//				//				std::cout<<"matches[corres_key.at(k)].queryIdx"<<matches[corres_key.at(k)].queryIdx<<std::endl;
//	//				Vector2d proj0(KeyframeDataVector.at(i).Keypoints[corres_key.at(k)].pt.x,KeyframeDataVector.at(i).Keypoints[corres_key.at(k)].pt.y);
//	//				//				std::cout<<"matches.size()"<<matches.size()<<std::endl;
//	//				//				std::cout<<"corres_sba.at(k)"<<corres_sba.at(k)<<std::endl;
//	//				sys.addMonoProj(node_counter,corres_sba.at(k),proj0);
//	//
//	//			}
//	////			KeyFramePoints.publish(sbacloud);
//	//			//		ROS_INFO("Sleeping for 5 seconds to publish sbacloud from keyframe %d.",i);
//	//			//		ros::Duration(5.0).sleep();
//	//			node_counter++;
//	//		}
//	//
//	//
//	//	}
//	std::cout<<"before graph"<<std::endl;
//	sba::drawGraph(sys,camSBA_marker_pub_preSBA,pointSBA_marker_pub_preSBA);
//	//
//	//	int npts=sys.tracks.size();
//	//
//	//	//		sys.doSBA(10);
//	//				sba::drawGraph(sys, cam_marker_pub_preSBA, point_marker_pub_preSBA);
//	//	ROS_INFO("Cameras (nodes): %d, Points: %d",
//	//			(int)sys.nodes.size(), (int)sys.tracks.size());
//	//
//	ROS_INFO("Sleeping for 5 seconds to publish pre-SBA markers.");
//	ros::Duration(5.0).sleep();
//	//
//	//
//	//	// Perform SBA with 10 iterations, an initial lambda step-size of 1e-3,
//	//	// and using CSPARSE.
//	///mainSLAM/points_preSBA	//		std::cout<<"before dsba"<<std::endl;
//	//	//		//			sys.doSBA(10);
//	//	//		sys.doSBA(10,10e-4,SBA_SPARSE_CHOLESKY);
//	//	//		int nbad = sys.removeBad(2.0);
//	//	//		int nremov=sys.removeBad(1.0);
//	//	//		int ntracks=sys.reduceTracks();
//	//	//		cout << endl << "Removed " << nbad << " projections > 2 pixels error" << "removed:"<<nremov<<"ntracks:"<<ntracks<<endl;
//	//	//		sys.doSBA(10,10e-5,SBA_DENSE_CHOLESKY);
//	//	std::cout<<"before sba"<<std::endl;
//	std::cout<<"before dosba"<<std::endl;
//
//	//		sys.doSBA(10);
//	int npts=sys.tracks.size();
//	//		npts=sys.tracks.size();
//	ROS_INFO("Bad projs (> 10 pix): %d, Cost without: %f",
//			(int)sys.countBad(10.0), sqrt(sys.calcCost(10.0)/npts));
//	ROS_INFO("Bad projs (> 5 pix): %d, Cost without: %f",
//			(int)sys.countBad(5.0), sqrt(sys.calcCost(5.0)/npts));
//	ROS_INFO("Bad projs (> 2 pix): %d, Cost without: %f",
//			(int)sys.countBad(2.0), sqrt(sys.calcCost(2.0)/npts));
//	//				sys.doSBA(5,10e-5,SBA_SPARSE_CHOLESKY);
//	std::cout<<"sys.tracks.size()"<<sys.tracks.size()<<std::endl;
//	std::cout<<"sys.tracks.at(2).projections.size()"<<sys.tracks.at(2).projections.size()<<std::endl;
//	std::cout<<"pointsWithProjectionTo2.size()"<<pointsWithProjectionTo2.size()<<std::endl;
//	std::vector<CvPoint> circle_vector;
//	for(uint i=0;i<pointsWithProjectionTo2.size()-1;i++)
//	{
//		circle.x=sys.tracks.at(pointsWithProjectionTo2.at(i)).projections.at(keyframeforreprojection).kp[0];
//		circle.y=sys.tracks.at(pointsWithProjectionTo2.at(i)).projections.at(keyframeforreprojection).kp[1];
//		cvCircle(track_reprojection_error,circle,3,colors[5],1,8,0);
//
//		Vector2d proj;
//
//		// Project the point into the node's image coordinate system.
//		sys.nodes[keyframeforreprojection].setProjection();
//		sys.nodes[keyframeforreprojection].project2im(proj, sys.tracks.at(pointsWithProjectionTo2.at(i)).point);
////		if(i<4)
////					std::cout<<"the point pos is:"<<sys.tracks.at(pointsWithProjectionTo2.at(i)).point<<std::endl;
//		std::cout<<"sys.tracks.at(i).projections.at(2)[1]"<<sys.tracks.at(pointsWithProjectionTo2.at(i)).projections.at(keyframeforreprojection).ndi<<std::endl;
//		circle.x=proj[0];
//		circle.y=proj[1];
//		circle_vector.push_back(circle);
//		cvCircle(track_reprojection_error,circle,1,colors[4],1,8,0);
//		//		std::cout<<"pointsWithProjectionTo2.at(i):"<<pointsWithProjectionTo2.at(i)<<std::endl;
//		//		std::cout<<"i:"<<i<<std::endl;
//	}
//
//	cvShowImage("reprojection error of image2",track_reprojection_error);
//	sba::drawGraph(sys,camSBA_marker_pub,pointSBA_marker_pub);
//
//	cvWaitKey(30);
//
//	std::cout<<"error of system: "<<sys.calcAvgError()<<std::endl;
//	ROS_INFO("Sleeping for 2 seconds to show image2 error.");
//	ros::Duration(2.0).sleep();
//	sys.removeBad(5.0);
//	std::cout<<"error of system after removed points: "<<sys.calcAvgError()<<std::endl;
//
//	sys.doSBA(5,10e-4,SBA_DENSE_CHOLESKY);
//	std::cout<<"error of system after sba: "<<sys.calcAvgError()<<std::endl;
//
////	  sys.removeBad(1.0);
////	  sys.doSBA(0);
//	//        cout << endl << "Removed " << nbad << " projections > 2 pixels error" << endl;
////	        sys.doSBA(5,10e-5,SBA_DENSE_CHOLESKY);//	int nbad = sys.removeBad(2.0);
////	int nremov=sys.removeBad(1.0);
////	int ntracks=sys.reduceTracks();
//	  		PointCloud SBAPointCloud;
//	  		SBAPointCloud.header.frame_id="/pgraph";
//	  		PointCloud TransPointCloud;
//	  		TransPointCloud.header.frame_id="/pgraph";
//	  for(uint i=0;i<KeyframeDataVector.size();i=i+5)
//	  {
//			//			translation=frametrans.block<3,1>(0,3);
//		  Eigen::Matrix4f transrotfloat;//=(Eigen::Matrix4f)transrot;
//
//		  Eigen::Matrix4d transrot;
//		  transrot.block<4,1>(0,3)=sys.nodes.at(i).trans;
//		  transrot.block<3,3>(0,0)=sys.nodes.at(i).qrot.matrix();
//
//		  std::cout<<"transrot"<<transrot<<std::endl<<std::endl;
//
//		  Vector4d tmpvec=transrot.block<4,1>(0,0);
//		  Vector4f tmpvecfloat;
//		  tmpvecfloat[0]=(float)tmpvec[0];
//		  tmpvecfloat[1]=(float)tmpvec[1];
//		  tmpvecfloat[2]=(float)tmpvec[2];
//		  tmpvecfloat[3]=0;//(float)tmpvec[3];
//		  transrotfloat.block<4,1>(0,0)=tmpvecfloat;
//
//
//		  tmpvec=transrot.block<4,1>(0,1);
//		  tmpvecfloat[0]=(float)tmpvec[0];
//		  tmpvecfloat[1]=(float)tmpvec[1];
//		  tmpvecfloat[2]=(float)tmpvec[2];
//		  tmpvecfloat[3]=0;//(float)tmpvec[3];
//		  transrotfloat.block<4,1>(0,1)=tmpvecfloat;
//
//		  tmpvec=transrot.block<4,1>(0,2);
//		  tmpvecfloat[0]=(float)tmpvec[0];
//		  tmpvecfloat[1]=(float)tmpvec[1];
//		  tmpvecfloat[2]=(float)tmpvec[2];
//		  tmpvecfloat[3]=0;//(float)tmpvec[3];
//		  transrotfloat.block<4,1>(0,2)=tmpvecfloat;
//
//		  tmpvec=transrot.block<4,1>(0,3);
//		  tmpvecfloat[0]=(float)tmpvec[0];
//		  tmpvecfloat[1]=(float)tmpvec[1];
//		  tmpvecfloat[2]=(float)tmpvec[2];
//		  tmpvecfloat[3]=1;//(float)tmpvec[3];
//		  transrotfloat.block<4,1>(0,3)=tmpvecfloat;
//
//		  std::cout<<"transrotfloat"<<transrotfloat<<std::endl<<transrot<<std::endl;
//
//
//
//
////		  transrotfloat.block<1,1>(0,0)=(float)tmpvec[0];
////		  		  transrotfloat.block<1,1>(0,1)=(float)tmpvec[1];
////		  		  transrotfloat.block<1,1>(0,2)=(float)tmpvec[2];
////		  		  transrotfloat.block<1,1>(0,3)=(float)tmpvec[3];
//		  //		  double tmp=transrot;
////		  transrotfloat.block<1,1>(0,0)=(float)tmp;
////		  transrotfloat.block<1,1>(0,1)=transrot.cast().block<1,1>(0,1);
////		  transrotfloat.block<1,1>(0,2)=transrot.block<1,1>(0,2);
////		  transrotfloat.block<1,1>(0,3)=transrot.block<1,1>(0,3);
////		  transrotfloat.block<1,1>(1,0)=transrot.block<1,1>(1,0);
////		  transrotfloat.block<1,1>(1,1)=transrot.block<1,1>(1,1);
////		  transrotfloat.block<1,1>(1,2)=transrot.block<1,1>(1,2);
////		  transrotfloat.block<1,1>(1,3)=transrot.block<1,1>(1,3);
////		  transrotfloat.block<1,1>(2,0)=transrot.block<1,1>(2,0);
////		  transrotfloat.block<1,1>(2,1)=transrot.block<1,1>(2,1);
////		  transrotfloat.block<1,1>(2,2)=transrot.block<1,1>(2,2);
////		  transrotfloat.block<1,1>(2,3)=transrot.block<1,1>(2,3);
////		  transrotfloat.block<1,1>(3,0)=transrot.block<1,1>(3,0);
////		  transrotfloat.block<1,1>(3,1)=transrot.block<1,1>(3,1);
////		  transrotfloat.block<1,1>(3,2)=transrot.block<1,1>(3,2);
////		  transrotfloat.block<1,1>(3,3)=transrot.block<1,1>(3,3);
////		  transrotfloat=(Eigen::MatrixBase<Eigen::Matrix<float,4,4,0,4,4> >)transrot;
//		  PointCloud sbatmp;
//		  sbatmp.header.frame_id="/pgraph";
//
//		  pcl::transformPointCloud(KeyframeDataVector.at(i).KinectCloud,sbatmp,transrotfloat);
//		  SBAPointCloud+=sbatmp;
//
//		  pcl::transformPointCloud(KeyframeDataVector.at(i).KinectCloud,sbatmp,KeyframeDataVector.at(i).Transformation);
//		  TransPointCloud+=sbatmp;
//
//		  std::cout<<"transformbefore:"<<KeyframeDataVector.at(i).Transformation<<std::endl;
//
//
//
//	  }
//
//
//
//	for(uint i=0;i<pointsWithProjectionTo2.size()-1;i++)
//		{
//			//		std::cout<<"sys.tracks.at(i).projections.at(2)[1]"<<sys.tracks.at(pointsWithProjectionTo2.at(i)).projections.at(2).ndi<<std::endl;
//			Vector2d proj;
//
//			// Project the point into the node's image coordinate system.
//			sys.nodes[keyframeforreprojection].setProjection();
//			sys.nodes[keyframeforreprojection].project2im(proj, sys.tracks.at(pointsWithProjectionTo2.at(i)).point);
//			std::cout<<"point: "<<sys.tracks.at(pointsWithProjectionTo2.at(i)).point<<std::endl;
//
//			//		std::cout<<"sys.tracks.at(i).projections.at(2)[1]"<<sys.tracks.at(pointsWithProjectionTo2.at(i)).projections.at(2).ndi<<std::endl;
//			CvPoint circle2;
//			circle2.x=proj[0];
//			circle2.y=proj[1];
//			std::cout<<"circle2.x\t"<<circle2.x<<"circle2.y\t"<<circle2.y<<std::endl;
//			std::cout<<"circle.x\t"<<circle_vector.at(i).x<<"circle.y\t"<<circle_vector.at(i).y<<std::endl;
//			if(circle2.y>-2000000)
//			cvLine( track_reprojection_error, circle_vector.at(i),circle2, colors[2] );
//
//	//		cvCircle(track_reprojection_error,circle,1,colors[4],1,8,0);
//			//		std::cout<<"pointsWithProjectionTo2.at(i):"<<pointsWithProjectionTo2.at(i)<<std::endl;
//			//		std::cout<<"i:"<<i<<std::endl;
//		}
//	cvShowImage("reprojection error of image2 after",track_reprojection_error);
//
//	for(uint i=0;i<sys.tracks.size();i++)
//	{
////		ProjMap::iterator it=sys.tracks.at(i).projections.begin();
//
//		uint size_of_tracks=sys.tracks.at(i).projections.size();
//		std::cout<<"sizeoftracks"<<size_of_tracks<<std::endl;
//		for(ProjMap::iterator it=sys.tracks.at(i).projections.begin();it!=sys.tracks.at(i).projections.end();++it)
//		{
//			std::cout<<"sys.tracks.at(i).projection.size()"<<sys.tracks.at(i).projections.size()<<std::endl;
//			std::cout<<"i:"<<i<<std::endl;
////			std::cout<<"d:"<<d<<std::endl;
//			std::cout<<"sys.tracks.size()"<<sys.tracks.size()<<std::endl;
//
//			std::cout<<".5"<<std::endl;
//			int actual_node=it->first;
//
//			std::cout<<"actual_node:"<<actual_node<<std::endl;
//			circle.x=sys.tracks.at(i).projections.at(actual_node).kp[0];
//			circle.y=sys.tracks.at(i).projections.at(actual_node).kp[1];
//			std::cout<<"1"<<std::endl;
//			cvCircle(track_vector.at(actual_node),circle,4,colors[2],1,8,0);
//			std::cout<<"2"<<std::endl;
//
//			Vector2d proj;
//
//			// Project the point into the node's image coordinate system.
//			sys.nodes[actual_node].setProjection();
//			sys.nodes[actual_node].project2im(proj, sys.tracks.at(i).point);
//			std::cout<<"3"<<std::endl;
//
//			//		if(i<4)
//			//					std::cout<<"the point pos is:"<<sys.tracks.at(pointsWithProjectionTo2.at(i)).point<<std::endl;
//			//		std::cout<<"sys.tracks.at(i).projections.at(2)[1]"<<sys.tracks.at(pointsWithProjectionTo2.at(i)).projections.at(2).ndi<<std::endl;
//			circle.x=proj[0];
//			circle.y=proj[1];
//			if(circle.y>-2000000)
//			cvCircle(track_vector.at(actual_node),circle,1,colors[4],1,8,0);
//			std::cout<<"4"<<std::endl;
//
//		}
//	}
//
//	for(uint i=0;i<track_vector.size();i++)
//	{
////		char name[10];
////		sprintf(name, "pic%d", i);
//////		name[3]=i;
////		std::cout<<"saving image:"<<name<<std::endl;
////		const char *nameini;
////		sprintf(*nameini, "pic%d", i);
//		std::stringstream sstr;
//		sstr << "aftersba"<<i;
//		std::string str1 = sstr.str();//std::string str2 = boost::lexical_cast<std::string>(my_int);
////		char name=(char)str1.c_str();
////		const char* name2=name;
//		//showallimages
////		cvShowImage(str1.c_str(),track_vector.at(i));
//		cvWaitKey(30);
//	}
//	cvWaitKey(0);
////	PointCloud sbaCloud;
////	sbaCloud.header.frame_id="/pgraph";
////	Point sbaPoint;
////	for(uint i=0;i<sys.tracks.size();i++)
////	{
////		if(!isnan(sys.tracks.at(i).point[0]))
////		{
////			sbaPoint.x=sys.tracks.at(i).point[2];
////			sbaPoint.y=-sys.tracks.at(i).point[0];
////			sbaPoint.z=-sys.tracks.at(i).point[1];
////			sbaCloud.push_back(sbaPoint);
////		}
////
////	}
////	SeenKeyFramePoints.publish(sbaCloud);
//
////	cout << endl << "Removed " << nbad << " projections > 2 pixels error" << "removed:"<<nremov<<"ntracks:"<<ntracks<<endl;
////	sys.doSBA(10,10e-5,SBA_DENSE_CHOLESKY);
////	for(uint i=0;i<pointsWithProjectionTo2.size()-1;i++)
////	{
////		//		std::cout<<"sys.tracks.at(i).projections.at(2)[1]"<<sys.tracks.at(pointsWithProjectionTo2.at(i)).projections.at(2).ndi<<std::endl;
////		Vector2d proj;
////
////		// Project the point into the node's image coordinate system.
////		sys.nodes[keyframeforreprojection].setProjection();
////		sys.nodes[keyframeforreprojection].project2im(proj, sys.tracks.at(pointsWithProjectionTo2.at(i)).point);
////
////
////		//		std::cout<<"sys.tracks.at(i).projections.at(2)[1]"<<sys.tracks.at(pointsWithProjectionTo2.at(i)).projections.at(2).ndi<<std::endl;
////		CvPoint circle2;
////		circle2.x=proj[0];
////		circle2.y=proj[1];
////		std::cout<<"circle2.x\t"<<circle2.x<<"circle2.y\t"<<circle2.y<<std::endl;
////		std::cout<<"circle.x\t"<<circle_vector.at(i).x<<"circle.y\t"<<circle_vector.at(i).y<<std::endl;
////
////		cvLine( track_reprojection_error, circle_vector.at(i),circle2, colors[2] );
////
//////		cvCircle(track_reprojection_error,circle,1,colors[4],1,8,0);
////		//		std::cout<<"pointsWithProjectionTo2.at(i):"<<pointsWithProjectionTo2.at(i)<<std::endl;
////		//		std::cout<<"i:"<<i<<std::endl;
////	}
////	cvShowImage("reprojection error of image2 after",track_reprojection_error);
////	cvWaitKey(0);
////	sba::drawGraph(sys,camSBA_marker_pub,pointSBA_marker_pub);
////
////	ROS_INFO("Sleeping for 2 seconds to show image2 error.");
////	ros::Duration(2.0).sleep();
////	sys.doSBA(1);
////	for(uint i=0;i<pointsWithProjectionTo2.size()-1;i++)
////	{
////		//		std::cout<<"sys.tracks.at(i).projections.at(2)[1]"<<sys.tracks.at(pointsWithProjectionTo2.at(i)).projections.at(2).ndi<<std::endl;
////		Vector2d proj;
////
////		// Project the point into the node's image coordinate system.
////		sys.nodes[keyframeforreprojection].setProjection();
////		sys.nodes[keyframeforreprojection].project2im(proj, sys.tracks.at(pointsWithProjectionTo2.at(i)).point);
////		//		std::cout<<"sys.tracks.at(i).projections.at(2)[1]"<<sys.tracks.at(pointsWithProjectionTo2.at(i)).projections.at(2).ndi<<std::endl;
////		if(i==0)
////					std::cout<<"the point pos is:"<<sys.tracks.at(pointsWithProjectionTo2.at(i)).point<<std::endl;
////		circle.x=proj[0];
////		circle.y=proj[1];
////		cvCircle(track_reprojection_error,circle,1,colors[4],1,8,0);
////		//		std::cout<<"pointsWithProjectionTo2.at(i):"<<pointsWithProjectionTo2.at(i)<<std::endl;
////		//		std::cout<<"i:"<<i<<std::endl;
////	}
////	cvShowImage("reprojection error of image2",track_reprojection_error);
////	cvWaitKey(30);
////	sba::drawGraph(sys,camSBA_marker_pub,pointSBA_marker_pub);
////
////	ROS_INFO("Sleeping for 2 seconds to show image2 error.");
////	ros::Duration(2.0).sleep();
////	sys.doSBA(1);
////	for(uint i=0;i<pointsWithProjectionTo2.size()-1;i++)
////	{
////		Vector2d proj;
////
////		// Project the point into the node's image coordinate system.
////		sys.nodes[keyframeforreprojection].setProjection();
////		sys.nodes[keyframeforreprojection].project2im(proj, sys.tracks.at(pointsWithProjectionTo2.at(i)).point);
////		if(i==0)
////			std::cout<<"the point pos is:"<<sys.tracks.at(pointsWithProjectionTo2.at(i)).point<<std::endl;
////		//		std::cout<<"sys.tracks.at(i).projections.at(2)[1]"<<sys.tracks.at(pointsWithProjectionTo2.at(i)).projections.at(2).ndi<<std::endl;
////		circle.x=proj[0];
////		circle.y=proj[1];
////		cvCircle(track_reprojection_error,circle,1,colors[4],1,8,0);
////		//		std::cout<<"pointsWithProjectionTo2.at(i):"<<pointsWithProjectionTo2.at(i)<<std::endl;
////		//		std::cout<<"i:"<<i<<std::endl;
////	}
////	cvShowImage("reprojection error of image2",track_reprojection_error);
////	cvWaitKey(30);
////	sba::drawGraph(sys,camSBA_marker_pub,pointSBA_marker_pub);
////
////	ROS_INFO("Sleeping for 2 seconds to show image2 error.");
////	ros::Duration(2.0).sleep();
//
//
//
//
//
//	while(ros::ok())
//	{
//
//		sba::drawGraph(sys,camSBA_marker_pub,pointSBA_marker_pub);
//
////publish pointcloud
//		std::cout<<"publishing pointclouds"<<std::endl;
////		KeyFramePoints.publish(SBAPointCloud);
//		transformedCloud.publish(TransPointCloud);
//
//		// Perform SBA with 10 iterations, an initial lambda step-size of 1e-3,
//		// and using CSPARSE.
//		//		std::cout<<"before dsba"<<std::endl;
//		//		//			sys.doSBA(10);
//		//		sys.doSBA(10,10e-4,SBA_SPARSE_CHOLESKY);
//		//		int nbad = sys.removeBad(2.0);
//		//		int nremov=sys.removeBad(1.0);
//		//		int ntracks=sys.reduceTracks();
//		//		cout << endl << "Removed " << nbad << " projections > 2 pixels error" << "removed:"<<nremov<<"ntracks:"<<ntracks<<endl;
//		//		sys.doSBA(10,10e-5,SBA_DENSE_CHOLESKY);
//
//
//		//		ROS_INFO("Bad projs (> 10 pix): %d, Cost without: %f",
//		//				(int)sys.countBad(10.0), sqrt(sys.calcCost(10.0)/npts));
//		//		ROS_INFO("Bad projs (> 5 pix): %d, Cost without: %f",
//		//				(int)sys.countBad(5.0), sqrt(sys.calcCost(5.0)/npts));
//		//		ROS_INFO("Bad projs (> 2 pix): %d, Cost without: %f",
//		//				(int)sys.countBad(2.0), sqrt(sys.calcCost(2.0)/npts));
//		//		KeyFramePoints.publish(afterSBAPointCloud);
//
//		ROS_INFO("Cameras (nodes): %d, Points: %d",
//				(int)sys.nodes.size(), (int)sys.tracks.size());
//		sba::drawGraph(sys,camSBA_marker_pub,pointSBA_marker_pub);
//		ROS_INFO("Sleeping for 5 seconds to publish post-SBA markers.");
//		ros::Duration(5.0).sleep();
//
//		std::cout<<"new trans:"<<std::endl<<sys.nodes[2].trans<<std::endl;
//		std::cout<<"new quaternion x:"<<sys.nodes[2].qrot.x()<<"y:"<<sys.nodes[0].qrot.y()<<"z:"<<sys.nodes[0].qrot.z()<<"w:"<<sys.nodes[0].qrot.w()<<std::endl;
//
//		//			std::cout<<"old trans:"<<std::endl<<sys.nodes[0].oldtrans<<std::endl;
//		//			std::cout<<"old quaternion x:"<<sys.nodes[0].oldqrot.x()<<"y:"<<sys.nodes[0].oldqrot.y()<<"z:"<<sys.nodes[0].oldqrot.z()<<"w:"<<sys.nodes[0].oldqrot.w()<<std::endl;
//
//	}
//}
//
//void EXTRACT::SBARANSAC(struct FrameData &Data,	vector<cv::DMatch> &matches, std::vector<int> &corresKey,std::vector<int> &corresSBA,int keyframenumber,PointCloud &sbacloud)
//{
//	Eigen::Vector3f tmp_dist;
//	//	float small_dist=10000;
//	//	uint keyframe_number=0;
//	std::vector<int> corr_Keyframe;
//	std::vector<int> corr_sba;
//	std::vector<int> corr_Keyframe_inliers;
//	std::vector<int> corr_Keyframe_inliers_tmp;
//	std::vector<int> corr_Keyframe_outliers;
//	PointCloud Feature_Keyframe[2];
//	Feature_Keyframe[0].header.frame_id="/pgraph";
//	Feature_Keyframe[1].header.frame_id="/pgraph";
//
//	cv::Mat tmpsbadesc;
//	tmpsbadesc.flags=dtors[counter].flags;
//	tmpsbadesc.dims=dtors[counter].dims;
//	tmpsbadesc.cols=dtors[counter].cols;
//	tmpsbadesc.step[0]=dtors[counter].step[0];
//	tmpsbadesc.step[1]=dtors[counter].step[1];
//	PointCloud tmpsbacloud;
//	tmpsbacloud.header.frame_id="/pgraph";
//
//	for(uint k=0;k<SBAmap.Descriptor.size();k++)
//	{
//		//			if(k!=keyframenumber)//&&k!=(keyframenumber-1)&&k!=(keyframenumber+1))
//		{
//			for(uint i=0;i<SBAmap.Descriptor.at(k).rows;i++)
//				tmpsbadesc.push_back(SBAmap.Descriptor.at(k).row(i));
//			tmpsbacloud+=SBAmap.Points.at(k);
//		}
//
//		if(k==keyframenumber)
//		{
//			std::cout<<"sbasize"<<SBAmap.Points.at(k).size()<<"keysize"<<Data.Points.size()<<std::endl;
//			std::cout<<"sbasize"<<SBAmap.Descriptor.at(k).rows<<"keysize"<<Data.Descriptor.rows<<std::endl;
//		}
//
//
//	}
//
//	sbacloud.header.frame_id="/pgraph";
//	sbacloud=tmpsbacloud;
//	matchFeature(Data.Descriptor,tmpsbadesc,matches);
//	std::cout<<"size of matches"<<matches.size()<<std::endl;
//	for (size_t i = 0; i < matches.size(); i++)
//	{
//		Feature_Keyframe[0].push_back(tmpsbacloud.points[matches[i].trainIdx]);
//		corr_Keyframe.push_back(i);
//		Feature_Keyframe[1].push_back(Data.Points.points[matches[i].queryIdx]);
//
//	}
//
//	//RANSAC
//
//
//	typedef pcl::SampleConsensusModelRegistration<Point>::Ptr SampleConsensusModelRegistrationPtr;
//
//	SampleConsensusModelRegistrationPtr model;
//
//
//	model.reset (new pcl::SampleConsensusModelRegistration<Point> (Feature_Keyframe[0].makeShared (), corr_Keyframe));
//	// Pass the target_indices
//
//
//	model->setInputTarget (Feature_Keyframe[1].makeShared(), corr_Keyframe);
//	// Create a RANSAC model
//
//	pcl::RandomSampleConsensus<Point> sac (model, .2);
//	sac.setMaxIterations (ransac_it);
//
//
//	// Compute the set of inliers
//
//	if (!sac.computeModel (5))
//	{
//		corr_Keyframe_inliers = corr_Keyframe;
//	}
//	else
//	{
//		std::vector<int> inliers;
//		// Get the inliers
//		sac.getInliers (inliers);
//		corr_Keyframe_inliers.resize (inliers.size ());
//		// Copy just the inliers
//		for (size_t i = 0; i < inliers.size (); ++i)
//		{
//			corr_Keyframe_inliers[i] = corr_Keyframe[inliers[i]];
//		}
//		//						std::vector<int> corr_Keyframe_in;
//		//						int c=0;
//		//						for (size_t i = 0; i < corr_Keyframe.size (); ++i)
//		//						{
//		//							if(i==inliers[c])
//		//							{
//		//								corr_Keyframe_in.push_back(corr_Keyframe[inliers[c]]);
//		//								c++;
//		//							}
//		//							else
//		//								corr_Keyframe_outliers.push_back(corr_Keyframe[i]);
//		//						}
//		//
//		//
//		//						std::cout<<"size of keyframeoriginal"<<corr_Keyframe_inliers.size()<<"size of keyframenew"<<corr_Keyframe_in.size()<<std::endl;
//		//						for(uint i=0;i<corr_Keyframe_inliers.size();i++)
//		//						{
//		//							std::cout<<"keyframeor"<<corr_Keyframe_inliers.at(i)<<"keyframenew"<<corr_Keyframe_in.at(i)<<std::endl;
//		//						}
//		//						for(uint i=0;i<corr_Keyframe_outliers.size();i++)
//		//							std::cout<<"outliers"<<corr_Keyframe_outliers.at(i)<<std::endl;
//
//	}
//
//	if(showDisplay)
//	{
//		ROS_ERROR("number of correspondences  after ransac with nearest keyframe %d",corr_Keyframe_inliers.size());
//	}
//	ransac_inliers=corr_Keyframe_inliers.size();
//
//	//		std::cout<<"before matches"<<std::endl;
//	//		matchFeature(Data.Descriptor,tmpsbadesc,matches);
//	//
//	//		for (size_t i = 0; i < matches.size(); i++)
//	//		{
//	//			corr_Keyframe.push_back(matches[i].queryIdx);
//	//			corr_sba.push_back(matches[i].trainIdx);
//	//
//	//		}
//	//		std::cout<<"size of pointcloud of keyframe"<<Data.Points.size()<<std::endl;
//	//		std::cout<<"size of matches of keyframe:"<<matches.size()<<std::endl;
//	//
//	//		//RANSAC
//	//
//	//
//	//		typedef pcl::SampleConsensusModelRegistration<Point>::Ptr SampleConsensusModelRegistrationPtr;
//	//
//	//		SampleConsensusModelRegistrationPtr model;
//	//
//	//		PointCloud tmp;
//	//		tmp.header.frame_id="/pgraph";
//	//		pcl::transformPointCloud(Data.Points,tmp,Data.Transformation);
//	//
//	//		model.reset (new pcl::SampleConsensusModelRegistration<Point> (tmp.makeShared (), corr_Keyframe));
//	//
//	//
//	//		model->setInputTarget (tmpsbacloud.makeShared(), corr_sba);
//	//		pcl::RandomSampleConsensus<Point> sac (model, .3);
//	//		sac.setMaxIterations (ransac_it);
//	//
//	//
//	//		// Compute the set of inliers
//	//
//	//		if (!sac.computeModel (5))
//	//		{
//	//			corr_Keyframe_inliers = corr_Keyframe;
//	//		}
//	//		else
//	//		{
//	//			std::vector<int> inliers;
//	//			// Get the inliers
//	//			sac.getInliers (inliers);
//	//			corresKey.clear();
//	//			corresSBA.clear();
//	//			//		corr_Keyframe_inliers.resize (inliers.size ());
//	//			// Copy just the inliers
//	corresKey.clear();
//	corresSBA.clear();
//	for (size_t i = 0; i < corr_Keyframe_inliers.size(); ++i)
//	{
//		corresKey.push_back(matches[corr_Keyframe_inliers[i]].queryIdx);
//		corresSBA.push_back(matches[corr_Keyframe_inliers[i]].trainIdx);
//	}
//	//
//	//		}
//	std::cout<<"size of inliers of keyframe:"<<corresKey.size()<<std::endl;
//
//	Eigen::Matrix4f sbatrans;
//	pcl::estimateRigidTransformationSVD(Feature_Keyframe[1],corr_Keyframe_inliers,Feature_Keyframe[0],corr_Keyframe_inliers,sbatrans);
//	std::cout<<"sbatrans"<<sbatrans<<std::endl;;
//
//}

void EXTRACT::viconCallback (const geometry_msgs::PoseStamped& viconMsg)
{
	std::cout<<"in viconcallback"<<std::endl;
	quat_vicon.x()=viconMsg.pose.orientation.x;
	quat_vicon.y()=viconMsg.pose.orientation.y;
	quat_vicon.z()=viconMsg.pose.orientation.z;
	quat_vicon.w()=viconMsg.pose.orientation.w;

	pos_vicon[0]=viconMsg.pose.position.x;
	pos_vicon[1]=viconMsg.pose.position.y;
	pos_vicon[2]=viconMsg.pose.position.z;

	//			Eigen::Matrix4f Rotz=Eigen::Matrix4f::Identity();
	//
	//			Rotz.col(0)[0]=cos(M_PI/2);
	//			Rotz.col(0)[1]=-sin(M_PI/2);
	//			Rotz.col(1)[0]=sin(M_PI/2);
	//			Rotz.col(1)[1]=cos(M_PI/2);
	//
	//		//	Eigen::Matrix4f Roty=Eigen::Matrix4f::Identity();
	//		//
	//		//	Roty.col(0)[0]=cos(-M_PI/2);
	//		//	Roty.col(0)[2]=sin(-M_PI/2);
	//		//	Roty.col(2)[0]=-sin(-M_PI/2);
	//		//	Roty.col(2)[2]=cos(-M_PI/2);
	//
	//			Eigen::Matrix4f Rotx=Eigen::Matrix4f::Identity();
	//			Rotx.col(1)[1]=cos(M_PI/2);
	//			Rotx.col(1)[2]=-sin(M_PI/2);
	//			Rotx.col(2)[1]=sin(M_PI/2);
	//			Rotx.col(2)[2]=cos(M_PI/2);
	//		Eigen::Matrix3f matrix(quat_vicon);
	//			btQuaternion tmp_quat(quat_vicon.x(),quat_vicon.y(),quat_vicon.z(),quat_vicon.w());
	//
	//			btMatrix3x3 m(tmp_quat);

	//			btMatrix3x3 m(q);
	//					std::cout<<"m:"<<m.getRow(0)[0]<<" "<<m.getRow(0)[1]<<" "<<m.getRow(0)[2]<<std::endl
	//							<<" "<<m.getRow(1)[0]<<" "<<m.getRow(1)[1]<<" "<<m.getRow(1)[2]<<std::endl
	//							<<" "<<m.getRow(2)[0]<<" "<<m.getRow(2)[1]<<" "<<m.getRow(2)[2]<<std::endl;
	//					double Roll, Pitch, Yaw;
	//					m.getRPY(Roll, Pitch, Yaw);


	Eigen::Quaternion<float> quat_vicon_eigen;
	quat_vicon_eigen.x()=-quat_vicon.y();
	quat_vicon_eigen.y()=-quat_vicon.z();
	quat_vicon_eigen.z()=quat_vicon.x();
	quat_vicon_eigen.w()=quat_vicon.w();


	btQuaternion quat_tmp(quat_vicon.y(),quat_vicon.z(),quat_vicon.x(),quat_vicon.w());


	//				quat_tmp=quat_vicon_eigen*quat_tmp;



	btMatrix3x3 m(quat_tmp);

	std::cout<<"m vicon:"<<m.getRow(0)[0]<<" "<<m.getRow(0)[1]<<" "<<m.getRow(0)[2]<<std::endl
			<<" "<<m.getRow(1)[0]<<" "<<m.getRow(1)[1]<<" "<<m.getRow(1)[2]<<std::endl
			<<" "<<m.getRow(2)[0]<<" "<<m.getRow(2)[1]<<" "<<m.getRow(2)[2]<<std::endl;




	vicontransform=Eigen::Matrix4f::Identity();
	//		Eigen::Matrix4f vicontransform;

	Eigen::Vector3f tmp_vec;

	tmp_vec[0]=m.getColumn(0)[0];
	tmp_vec[1]=m.getColumn(0)[1];
	tmp_vec[2]=m.getColumn(0)[2];

	vicontransform.block<3,1>(0,0)=tmp_vec;

	tmp_vec[0]=m.getColumn(1)[0];
	tmp_vec[1]=m.getColumn(1)[1];
	tmp_vec[2]=m.getColumn(1)[2];

	vicontransform.block<3,1>(0,1)=tmp_vec;

	tmp_vec[0]=m.getColumn(2)[0];
	tmp_vec[1]=m.getColumn(2)[1];
	tmp_vec[2]=m.getColumn(2)[2];

	vicontransform.block<3,1>(0,2)=tmp_vec;

	tmp_vec[0]=pos_vicon[1];
	tmp_vec[1]=pos_vicon[2];
	tmp_vec[2]=pos_vicon[0];

	vicontransform.block<3,1>(0,3)=tmp_vec;



}

void EXTRACT::commandCallback (const lcm_mavlink_ros::COMMAND& commandMsg)
{
	std::cout<<"in commandcallback"<<std::endl;
	std::cout<<"commandmsg.command:"<<commandMsg.command<<std::endl;
	if(commandMsg.command==200)
		take_vicon=true;
	if(commandMsg.command==241)
		reset_map=true;
}


void EXTRACT::imuCallback (const sensor_msgs::Imu& imuMsg)
{
	if(notcopied)
	{
		//imuMutex_.lock();

		quat_imu.x()=imuMsg.orientation.x;
		quat_imu.y()=imuMsg.orientation.y;
		quat_imu.z()=imuMsg.orientation.z;
		quat_imu.w()=imuMsg.orientation.w;

		btQuaternion q(-imuMsg.orientation.y, -imuMsg.orientation.z,imuMsg.orientation.x,  imuMsg.orientation.w);
		//		btQuaternion q(imuMsg.orientation.x, -imuMsg.orientation.y,imuMsg.orientation.z,  imuMsg.orientation.w);
		btMatrix3x3 m(q);
		double Roll, Pitch, Yaw;
		m.getRPY(Roll, Pitch, Yaw);
		//	m.setRPY(Roll,Pitch,0);

		std::cout<<"roll"<<Roll<<"pitch"<<Pitch<<"yaw"<<Yaw<<std::endl;



		Eigen::Vector3f tmp_vec;

		tmp_vec[0]=m.getColumn(0)[0];
		tmp_vec[1]=m.getColumn(0)[1];
		tmp_vec[2]=m.getColumn(0)[2];

		imuRot.block<3,1>(0,0)=tmp_vec;

		tmp_vec[0]=m.getColumn(1)[0];
		tmp_vec[1]=m.getColumn(1)[1];
		tmp_vec[2]=m.getColumn(1)[2];

		imuRot.block<3,1>(0,1)=tmp_vec;

		tmp_vec[0]=m.getColumn(2)[0];
		tmp_vec[1]=m.getColumn(2)[1];
		tmp_vec[2]=m.getColumn(2)[2];

		imuRot.block<3,1>(0,2)=tmp_vec;

		notcopied=false;




		//	std::cout<<"roll: "<<Roll<<"pitch: "<<Pitch<<"yaw:"<<Yaw<<std::endl;
		//
		//	Yaw=0;
		//
		//	Eigen::Matrix4f RotXRoll=Eigen::Matrix4f::Identity();
		//	RotXRoll.col(1)[1]=cos(-Roll);
		//	RotXRoll.col(1)[2]=sin(-Roll);
		//	RotXRoll.col(2)[1]=-sin(-Roll);
		//	RotXRoll.col(2)[2]=cos(-Roll);
		//
		////	std::cout<<"rollxroll\n"<<RotXRoll<<std::endl;
		//
		//	Eigen::Matrix4f RotYPitch=Eigen::Matrix4f::Identity();
		//
		//	RotYPitch.col(0)[0]=cos(Pitch);
		//	RotYPitch.col(0)[2]=-sin(Pitch);
		//	RotYPitch.col(2)[0]=sin(Pitch);
		//	RotYPitch.col(2)[2]=cos(Pitch);
		////	std::cout<<"rollpitch\n"<<RotYPitch<<std::endl;
		//
		//
		//	Eigen::Matrix4f RotZYaw=Eigen::Matrix4f::Identity();
		//
		//	RotZYaw.col(0)[0]=cos(-Yaw);
		//	RotZYaw.col(0)[1]=sin(-Yaw);
		//	RotZYaw.col(1)[0]=-sin(-Yaw);
		//	RotZYaw.col(1)[1]=cos(-Yaw);
		//	std::cout<<"rollyaw\n"<<RotZYaw<<std::endl;
		//
		//	imuRot=RotYPitch.inverse()*imuRot;
		//
		//
		//
		//	Eigen::Matrix4f Rotz=Eigen::Matrix4f::Identity();
		//
		//	Rotz.col(0)[0]=cos(M_PI/2);
		//	Rotz.col(0)[1]=-sin(M_PI/2);
		//	Rotz.col(1)[0]=sin(M_PI/2);
		//	Rotz.col(1)[1]=cos(M_PI/2);

		//	Eigen::Matrix4f Roty=Eigen::Matrix4f::Identity();
		//
		//	Roty.col(0)[0]=cos(-M_PI/2);
		//	Roty.col(0)[2]=sin(-M_PI/2);
		//	Roty.col(2)[0]=-sin(-M_PI/2);
		//	Roty.col(2)[2]=cos(-M_PI/2);

		//	Eigen::Matrix4f Rotx=Eigen::Matrix4f::Identity();
		//	Rotx.col(1)[1]=cos(M_PI/2);
		//	Rotx.col(1)[2]=-sin(M_PI/2);
		//	Rotx.col(2)[1]=sin(M_PI/2);
		//	Rotx.col(2)[2]=cos(M_PI/2);





		//	btQuaternion q2;
		//	q2.setRPY(Raw2, Pitch2, 0.0);
		//	curr_rotation.setRotation(q2);				//no information about Yaw rotation

		//	btTransform change_rot = curr_rotation*prev_rotation.inverse();					//prev_rotation.inverse()*curr_rotation;
		//	btQuaternion temp_rot(change_rot.getRotation());

		//	btMatrix3x3 m2(temp_rot);
		//	double Raw2, Pitch2, Yaw2;
		//	m2.getRPY(Raw2, Pitch2, Yaw2);
		//1st
		//	imuRot=RotXRoll*RotYPitch*RotZYaw;//Rotz*Rotx*RotXRoll*RotYPitch*RotZYaw;//Eigen::Matrix4f::Identity();
		//	std::cout<<"imurot"<<std::endl<<imuRot<<std::endl;
		//	imuRot=Eigen::Matrix4f::Identity();
		//	imuRot.col(0)[0]=(float)cos(Pitch);
		//	imuRot.col(0)[1]=(float)sin(Pitch)*sin(Roll);
		//	imuRot.col(0)[2]=(float)sin(Pitch)*cos(Roll);
		//	imuRot.col(1)[1]=(float)cos(Roll);
		//	imuRot.col(1)[2]=(float)-sin(Roll);
		//	imuRot.col(2)[0]=(float)-sin(Pitch);
		//	imuRot.col(2)[1]=(float)cos(Pitch)*sin(Roll);
		//	imuRot.col(2)[2]=(float)cos(Pitch)*cos(Roll);


		//	( 	cos(Pitch2),	sin(Pitch2)*sin(Raw2),	sin(Pitch2)*cos(Raw2),	0,
		//				0,		cos(Raw2),		-sin(Raw2),		0,
		//				-sin(Pitch2),	cos(Pitch2)*sin(Raw2),	cos(Pitch2)*cos(Raw2),	0,
		//				0,		0,			0,			1);

		/*
	//2nd
	change_rotation_ << 	0,		-cos(Raw2),		sin(Raw2),		0,
				-sin(Pitch2),	cos(Pitch2)*sin(Raw2),	cos(Pitch2)*cos(Raw2),	0,
				-cos(Pitch2),	-sin(Pitch2)*sin(Raw2),	-sin(Pitch2)*cos(Raw2),	0,
				0,		0,			0,			1;

	change_rotation_ << 	-sin(Pitch2),	cos(Pitch2)*sin(Raw2),	cos(Pitch2)*cos(Raw2),	0,
				-cos(Pitch2),	-sin(Pitch2)*sin(Raw2),	-sin(Raw2),		0,
				0,		-cos(Raw2),		-sin(Pitch2)*cos(Raw2),	0,
				0,		0,			0,			1;


	change_rotation_ << 	0,		-cos(Raw2),		sin(Raw2),		0,
				sin(Pitch2),	-cos(Pitch2)*sin(Raw2),	-cos(Pitch2)*cos(Raw2),	0,
				cos(Pitch2),	sin(Pitch2)*sin(Raw2),	sin(Pitch2)*cos(Raw2),	0,
				0,		0,			0,			1;
		 */
		//	cout<<"change_rotation "<<endl;
		//	cout<<imuRot<<endl;
		//	cout<<"Raw "<<Raw2<<" Pitch "<<Pitch2<<" Yaw "<< Yaw2<<endl;

		//	prev_rotation = curr_rotation;
		//imuMutex_.unlock();
	}
}

