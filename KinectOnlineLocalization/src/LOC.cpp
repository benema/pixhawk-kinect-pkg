#include "LOC.h"

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

LOC::LOC(float thresh, int iterations,int minimal_inliers, int keyframe_inliers,bool verbose, int type_of_odometry,string filepath, float z1, float z2, float r1, float r2)
{
	//Initialization of start parameters
	path=filepath;
	type=type_of_odometry;
	showDisplay=verbose;
	ransac_acc=thresh;
	ransac_it=iterations;
	min_inliers=minimal_inliers;
	min_keyframe_inlier=keyframe_inliers;
	dist_z1=z1;
	dist_z2=z2;
	radius1=r1;
	radius2=r2;

	//other variables
	detector = new cv::GridAdaptedFeatureDetector(new cv::FastFeatureDetector,500);
	extractor=new cv::BriefDescriptorExtractor;
	called=false;
	counter=0;
	called_first_time=true;
	cv_image[0]=cvCreateImage(cvSize(640,480),8,3);
	cv_image[1]=cvCreateImage(cvSize(640,480),8,3);
	imgadd=cvCreateImage( cvSize(640, 960), IPL_DEPTH_8U, 3 );
	callback_image=cvCreateImage(cvSize(640,480),8,3);
	compute_transform_success=true;
	transformation_at_least_once_computed=false;
	stop_mapping=false;
	vicon_computed=false;
	next_keyframe=false;
	FeaturePointCloud[0].header.frame_id=std::string("/openni_depth_optical_frame");
	FeaturePointCloud[0].is_dense = false;
	FeaturePointCloud[1].header.frame_id=std::string("/openni_depth_optical_frame");
	FeaturePointCloud[1].is_dense = false;
	transformation_at_least_twice_computed=false;
	actual_keyframe=0;
	take_vicon=false;
	vicontransform=Eigen::Matrix4f::Identity();
	map_loaded=false;
	computed_initial_position=false;



	ros::NodeHandle n;


	//time the incoming rgb and cloud msgs
	message_filters::Subscriber<PointCloud> pointCloud_sub(n, "/camera/depth/points", 1);
	message_filters::Subscriber<sensor_msgs::Image> rgbImage_sub(n, "/camera/rgb/image_color", 1);
	typedef message_filters::sync_policies::ApproximateTime<PointCloud, sensor_msgs::Image> MySyncPolicy;
	// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pointCloud_sub, rgbImage_sub);



	//Callback functions for MAVLINK stuff
	imuSubscriber = n.subscribe ("/fromMAVLINK/Imu",  1, &LOC::imuCallback,  this);
	viconSubscriber= n.subscribe("/fromMAVLINK/Vicon",1,&LOC::viconCallback,this);
	commandSubscriber= n.subscribe("/fromMAVLINK/COMMAND",1,&LOC::commandCallback,this);


	KeyFramePoints=n.advertise<PointCloud>("KinectOnlineLocalization/keyframepoints",1);
	BubbleCenter=n.advertise<PointCloud>("KinectOnlineLocalization/bubbles",1);
	cameraPose_pub=n.advertise<geometry_msgs::PoseStamped>("/KinectOnlineLocalization/helicopterpose",1);
	bodyPoseStamped_pub=n.advertise<geometry_msgs::PoseStamped>("/toMAVLINK/bodyPoseStamped",1);






	sync.registerCallback(&LOC::callback,this);
	//	ros::spin();



	ros::AsyncSpinner spinner(0);
	spinner.start();
	while(ros::ok())
	{
		if(type==0)
		{
			if(map_loaded==false)
			{
				std::ifstream readfile;
				readfile.open(path.c_str(),ios::binary);
				if(readfile.is_open())
					std::cout<<"file is open"<<std::endl;
				else
				{
					std::cout<<"couldn't open file"<<std::endl;
					return;
				}
				int pointsize;
				readfile.read((char *)(&pointsize),sizeof(pointsize));
				std::cout<<"pointsize:"<<pointsize<<std::endl;
				Point tmpPoint;
				for(uint i=0;i<pointsize;i++)
				{
					readfile.read((char *)(&tmpPoint.x),sizeof(tmpPoint.x));
					readfile.read((char *)(&tmpPoint.y),sizeof(tmpPoint.y));
					readfile.read((char *)(&tmpPoint.z),sizeof(tmpPoint.z));
					//					std::cout<<"tmpPoint.z"<<tmpPoint.z<<std::endl;
					copiedmap.Points.push_back(tmpPoint);
				}
				int rows,cols;

				readfile.read((char *)(&rows),sizeof(rows));
				readfile.read((char *)(&cols),sizeof(cols));

				cv::Mat tmp_desc(rows,cols,0);
				for(uint y=0;y<rows;y++)
					for(uint x=0;x<cols;x++)
						readfile.read((char *)(tmp_desc.row(y).col(x).data),sizeof(*tmp_desc.row(y).col(x).data));

				readfile.close();

				copiedmap.Descriptor=tmp_desc;
				copiedmap.Points.header.frame_id="/pgraph";
				//				std::ifstream readfile;
				//				readfile.open(path.c_str(),ios::binary);
				//				if(readfile.is_open())
				//					std::cout<<"file is open"<<std::endl;
				//				else
				//				{
				//					std::cout<<"couldn't open file"<<std::endl;
				//					return;
				//				}
				//				readfile.read((char *)(&copiedmap),sizeof(copiedmap));
				//				readfile.close();
				map_loaded=true;

				//				std::cout<<"size of points"<<copiedmap.Points.size()<<std::endl;

				kdtree.setInputCloud(copiedmap.Points.makeShared());

				if(showDisplay)
				{
					std::cout<<"map copied"<<std::endl;
					//					std::cout<<"copiedmap.pooints.x"<<copiedmap.Points.points.at(0).x<<std::endl;
					//					std::cout<<"copiedmap.descripotr:"<<copiedmap.Descriptor<<std::endl;
				}

			}
			if(called==1&&map_loaded)
			{
				if(showDisplay)
					start=clock();

				cvCopy(callback_image,cv_image[0]);
				if(showDisplay)
				{
					cvShowImage("RGB Image 1", cv_image[0]);
					cvWaitKey(30);

				}
				kinectCloud[0]=callbackCloud;
				kinectCloud[0].header.frame_id="/pgraph";
				called=0;
				PointCloud tmpCloud;
				tmpCloud.header.frame_id="/pgraph";
				std::vector<cv::KeyPoint> kpts_tmp;
				Point tmpPoint;
				cv::Mat dtorstmp;
				detector->detect(cv_image[0],kpts[0]);
				extractor->compute(cv_image[0],kpts[0],dtors[0]);
				dtorstmp.flags=dtors[0].flags;
				dtorstmp.dims=dtors[0].dims;
				dtorstmp.cols=dtors[0].cols;
				dtorstmp.step[0]=dtors[0].step[0];
				dtorstmp.step[1]=dtors[0].step[1];


				for(uint i=0;i<kpts[0].size();i++)
				{
					if(	pcl_isfinite (kinectCloud[0].at(kpts[0].at(i).pt.x,kpts[0].at(i).pt.y).x) &&
							pcl_isfinite (kinectCloud[0].at(kpts[0].at(i).pt.x,kpts[0].at(i).pt.y).y) &&
							pcl_isfinite (kinectCloud[0].at(kpts[0].at(i).pt.x,kpts[0].at(i).pt.y).z))
					{
						tmpPoint=kinectCloud[0].at(kpts[0].at(i).pt.x,kpts[0].at(i).pt.y);
						dtorstmp.push_back(dtors[0].row(i));
						kpts_tmp.push_back(kpts[0].at(i));
						tmpCloud.push_back(tmpPoint);

					}
				}

				dtors[0]=dtorstmp;
				kpts[0]=kpts_tmp;

				FrameData[0].Points=tmpCloud;
				FrameData[0].Descriptor=dtors[0];
				FrameData[0].Keypoints=kpts[0];
				FrameData[0].KinectCloud=kinectCloud[0];

				int ransacInlierToMap;
				int ransacInlierToFrame;

				//do RANSAC and compute the transforamtion the first time
				if(computed_initial_position==false)
				{
					computeTransformationToMap(FrameData[0],copiedmap, transformation_,ransacInlierToMap);

					if(ransacInlierToMap>min_inliers)
					{
						computed_initial_position=true;
						swapSingleFrame();
					}
					else
					{
						if(showDisplay)
							std::cout<<"couldn't initialize on map..."<<std::endl;
					}
				}
				else
				{
					Eigen::Matrix4f estimated_trans;
					//estimate the transformation

					//					std::cout<<"FrameData[1].Descriptor.size"<<FrameData[1].Descriptor.size<<std::endl;
					computeTransformationBetweenFrameData(FrameData[0],FrameData[1],estimated_trans,ransacInlierToFrame);
					//					estimated_trans=Eigen::Matrix4f::Identity();



					if(ransacInlierToFrame>min_inliers)
					{
						transformation_=estimated_trans*transformation_;
						swapSingleFrame();
					}
					else
						if(showDisplay)
							ROS_ERROR("ransacInlierToFrame: %d", ransacInlierToFrame);

					if(showDisplay)
						std::cout<<"starttransform:"<<transformation_<<std::endl;


					//computing the indices in front of the camera
					if(showDisplay)
						start_match=clock();

					struct MapData tmp_map;

					PointCloud searchpoints;
					searchpoints.header.frame_id="/pgraph";

					computePointsWithin2Circles(kdtree, dist_z1,dist_z2, radius1, radius2, transformation_, copiedmap, tmp_map,searchpoints);

					if(showDisplay)
					{
						end_match=clock();
						std::cout<<"compute indices takes"<<(double(end_match)-double(start_match))/double(CLOCKS_PER_SEC);


						tmp_map.Points.header.frame_id="/pgraph";

						std::cout<<"size of indices"<<tmp_map.Points.size()<<std::endl;

						KeyFramePoints.publish(tmp_map.Points);
						BubbleCenter.publish(searchpoints);
					}

					Eigen::Matrix4f transformToMap;
					computeTransformationToMap(FrameData[0],tmp_map, transformToMap,ransacInlierToMap);

					if(ransacInlierToMap>min_inliers)
					{
						transformation_=transformToMap;
						swapSingleFrame();
					}
					else
					{
						if(showDisplay)
						{
							ROS_ERROR("ransacInlierToMap: %d",ransacInlierToMap);
							ROS_WARN("ransacInlierToFrame: %d", ransacInlierToFrame);
							computeTransformationToMap(FrameData[0],copiedmap,transformToMap,ransacInlierToMap);
							ROS_ERROR("ransacInlierToWholeMap: %d",ransacInlierToMap);

						}
					}


				}
				if(showDisplay)
					std::cout<<"endtransform:"<<transformation_<<std::endl;



				Eigen::Matrix3f rot_matrix=transformation_.block<3,3>(0,0);
				Eigen::Vector3f trans_vec=transformation_.block<3,1>(0,3);
				Eigen::Quaternion<float> quat_rot(rot_matrix);

				// variable for the position of the camera (used only for visualization)
				geometry_msgs::PoseStamped cameraPose;

				if(showDisplay)
				{


					cameraPose.header.frame_id="/pgraph";
					cameraPose.pose.position.x=trans_vec[0];
					cameraPose.pose.position.y=trans_vec[1];
					cameraPose.pose.position.z=trans_vec[2];

					cameraPose.pose.orientation.w=quat_rot.w();
					cameraPose.pose.orientation.x=quat_rot.x();
					cameraPose.pose.orientation.y=quat_rot.y();
					cameraPose.pose.orientation.z=quat_rot.z();

					cameraPose_pub.publish(cameraPose);
					//					KeyFramePoints.publish(copiedmap.Points);
				}


				geometry_msgs::PoseStamped heliPose;

				heliPose.header.frame_id="/pgraph";
				heliPose.header.stamp=ros::Time::now();
				heliPose.pose.position.x=trans_vec[2];
				heliPose.pose.position.y=trans_vec[0];
				heliPose.pose.position.z=trans_vec[1];

				heliPose.pose.orientation.w=quat_rot.w();
				heliPose.pose.orientation.x=quat_rot.z();
				heliPose.pose.orientation.y=quat_rot.x();
				heliPose.pose.orientation.z=quat_rot.y();


				bodyPoseStamped_pub.publish(heliPose);


				if(showDisplay)
				{
					end=clock();
					std::cout<<"time to compute transformation"<<(double(end)-double(start))/double(CLOCKS_PER_SEC);
				}
			}
		}

		if(type==1)
			if(called==1)
			{


				if(called_first_time)
					counter=0;
				else
					counter=1;

				cvCopy(callback_image,cv_image[counter]);
				kinectCloud[counter]=callbackCloud;
				kinectCloud[counter].header.frame_id="/pgraph";
				called=0;
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

				if(showDisplay)
					std::cout<<"2"<<std::endl;

				for(uint i=0;i<kpts[counter].size();i++)
				{
					if(	pcl_isfinite (kinectCloud[counter].at(kpts[counter].at(i).pt.x,kpts[counter].at(i).pt.y).x) &&
							pcl_isfinite (kinectCloud[counter].at(kpts[counter].at(i).pt.x,kpts[counter].at(i).pt.y).y) &&
							pcl_isfinite (kinectCloud[counter].at(kpts[counter].at(i).pt.x,kpts[counter].at(i).pt.y).z))
					{
						tmpPoint=kinectCloud[counter].at(kpts[counter].at(i).pt.x,kpts[counter].at(i).pt.y);
						dtorstmp.push_back(dtors[counter].row(i));
						kpts_tmp.push_back(kpts[counter].at(i));
						tmpCloud.push_back(tmpPoint);

					}
				}
				;
				dtors[counter]=dtorstmp;
				kpts[counter]=kpts_tmp;

				FrameData[counter].Points=tmpCloud;
				FrameData[counter].Descriptor=dtors[counter];
				FrameData[counter].Keypoints=kpts[counter];
				FrameData[counter].KinectCloud=kinectCloud[counter];

				if(counter==0)
				{
					if(showDisplay)
					{
						std::cout<<"KeyframeDataVector.size()"<<KeyframeDataVector.size()<<std::endl;
					}
					KeyframeDataVector.push_back(FrameData[counter]);
					KeyframeDataVector.at(0).Transformation=Eigen::Matrix4f::Identity();
					if(showDisplay)
					{
						std::cout<<"KeyframeDataVector..trans"<<KeyframeDataVector.at(0).Transformation<<std::endl;
					}
				}

				called_first_time=false;

				if(counter==1)
				{
					if(showDisplay)
					{
						cvSetImageROI( imgadd,cvRect( 0, 0,cv_image[0]->width, cv_image[0]->height ) );
						cvCopy(cv_image[0],imgadd);
						cvResetImageROI(imgadd);
						cvSetImageROI( imgadd,cvRect( 0, cv_image[0]->height,cv_image[0]->width, cv_image[0]->height ) );
						cvCopy(cv_image[1],imgadd);
						cvResetImageROI(imgadd);
					}



					matchFeature(dtors[0],dtors[1],matches_popcount);

					if(showDisplay)
						std::cout<<"5"<<std::endl;
					if(showDisplay)
						std::cout<<"size of matches_popcount"<<matches_popcount.size()<<std::endl;


					//do RANSAC and compute the transforamtion
					RANSACandTransformation();



					if(next_keyframe)
					{
						swap();

					}
					if(showDisplay)
					{
						cvShowImage("RGB Image 1", imgadd);
						cvWaitKey(30);

					}


				}

			}
	}

	spinner.stop();
	cvReleaseImage(&imgadd);
	cvReleaseImage(&cv_image[0]);
	cvReleaseImage(&cv_image[1]);

}


void LOC::viconCallback (const geometry_msgs::PoseStamped& viconMsg)
{
	if(showDisplay)
		std::cout<<"in viconcallback"<<std::endl;
	quat_vicon.x()=viconMsg.pose.orientation.x;
	quat_vicon.y()=viconMsg.pose.orientation.y;
	quat_vicon.z()=viconMsg.pose.orientation.z;
	quat_vicon.w()=viconMsg.pose.orientation.w;

	pos_vicon[0]=viconMsg.pose.position.x;
	pos_vicon[1]=viconMsg.pose.position.y;
	pos_vicon[2]=viconMsg.pose.position.z;


	Eigen::Quaternion<float> quat_vicon_eigen;
	quat_vicon_eigen.x()=-quat_vicon.y();
	quat_vicon_eigen.y()=-quat_vicon.z();
	quat_vicon_eigen.z()=quat_vicon.x();
	quat_vicon_eigen.w()=quat_vicon.w();


	btQuaternion quat_tmp(quat_vicon.y(),quat_vicon.z(),quat_vicon.x(),quat_vicon.w());


	btMatrix3x3 m(quat_tmp);
	if(showDisplay)
		std::cout<<"m vicon:"<<m.getRow(0)[0]<<" "<<m.getRow(0)[1]<<" "<<m.getRow(0)[2]<<std::endl
		<<" "<<m.getRow(1)[0]<<" "<<m.getRow(1)[1]<<" "<<m.getRow(1)[2]<<std::endl
		<<" "<<m.getRow(2)[0]<<" "<<m.getRow(2)[1]<<" "<<m.getRow(2)[2]<<std::endl;




	vicontransform=Eigen::Matrix4f::Identity();

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

	vicon_computed=true;



}

void LOC::commandCallback (const lcm_mavlink_ros::COMMAND& commandMsg)
{
	if(commandMsg.command==200)
		take_vicon=true;
	if(commandMsg.command==201)
		stop_mapping=true;
}


void LOC::imuCallback (const sensor_msgs::Imu& imuMsg)
{
	quat_imu.x()=imuMsg.orientation.x;
	quat_imu.y()=imuMsg.orientation.y;
	quat_imu.z()=imuMsg.orientation.z;
	quat_imu.w()=imuMsg.orientation.w;

	btQuaternion q(-imuMsg.orientation.y, -imuMsg.orientation.z,imuMsg.orientation.x,  imuMsg.orientation.w);
	btMatrix3x3 m(q);
	double Roll, Pitch, Yaw;
	m.getRPY(Roll, Pitch, Yaw);
	//	m.setRPY(Roll,Pitch,0);
	if(showDisplay)
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
}

void LOC::callback(const PointCloud::ConstPtr& pointCloud_ptr,const sensor_msgs::ImageConstPtr& image_ptr)
{
	if(showDisplay)
		std::cout<<"in kinectcallback"<<std::endl;
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

}
void LOC::matchFeature(cv::Mat &dtors0,cv::Mat&dtors1,	vector<cv::DMatch> &matches)
{
	matcher_popcount.match(dtors0, dtors1, matches);
}

void LOC::RANSACandTransformation()
{
	actual_keyframe=KeyframeDataVector.size()-1;

	createCorrespondingPointcloud(FrameData[0],FeaturePointCloud[0],FrameData[1],FeaturePointCloud[1],correspondences,matches_popcount,showDisplay,kpts[0],kpts[1]);

	RANSAC();

	if(showDisplay)
		std::cout<<"transformOld"<<std::endl<<transformOld<<std::endl;

	FrameData[counter].Transformation=transformOld;


	if(showDisplay)
	{

		{
			if(actual_keyframe==(KeyframeDataVector.size()-1))
			{
				if(showDisplay)
					ROS_WARN("in cvline target_good");

				for(uint i=0;i<correspondences_source_good.size();i++)
				{
					cvLine( imgadd, cvPoint(kpts[0][matches_popcount[correspondences_source_good[i]].queryIdx].pt.x, kpts[0][matches_popcount[correspondences_source_good[i]].queryIdx].pt.y),
							cvPoint(kpts[1][matches_popcount[correspondences_source_good[i]].trainIdx].pt.x, 480+kpts[1][matches_popcount[correspondences_source_good[i]].trainIdx].pt.y), colors[2] );
				}
			}
		}
	}


	if(showDisplay)
		std::cout<<"correspondences_inliers.size() RANSACINLIERS"<<correspondences_inliers.size()<<std::endl;
}





void LOC::createCorrespondingPointcloud(struct FrameData& Data0,PointCloud& Cloud0,struct FrameData& Data1,PointCloud& Cloud1,std::vector<int>& correspondvector,vector<cv::DMatch>& matches,bool show,std::vector<cv::KeyPoint>& kpts0,std::vector<cv::KeyPoint>& kpts1)

{

	std::vector<int> corr_tmp;
	std::vector<int> corr_matches_tmp;

	PointCloud FeatureTmp[2];
	FeatureTmp[0].header.frame_id="/pgraph";
	FeatureTmp[1].header.frame_id="/pgraph";



	Point push_back_point;

	{
		if(showDisplay)
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

//code partially copied from http://www.cs.unc.edu/~blloyd/comp290-089/fmatrix/
void LOC::RANSAC()
{
	if(showDisplay)
		std::cout<<"inransac"<<std::endl;


	Eigen::Matrix4f frametrans=Eigen::Matrix4f::Identity();
	correspondences_source_good.clear();
	correspondences_target_good.clear();
	Eigen::Vector3f translation;


	/***Defining all the RANSAC STUFF***/


	typedef pcl::SampleConsensusModelRegistration<Point>::Ptr SampleConsensusModelRegistrationPtr;

	SampleConsensusModelRegistrationPtr model;


	model.reset (new pcl::SampleConsensusModelRegistration<Point> (FeaturePointCloud[1].makeShared (), correspondences));
	// Pass the target_indices

	model->setInputTarget (FeaturePointCloud[0].makeShared(), correspondences);
	// Create a RANSAC model

	pcl::RandomSampleConsensus<Point> sac (model, ransac_acc);
	sac.setMaxIterations (ransac_it);


	// Compute the set of inliers

	if (!sac.computeModel (5))
	{
		correspondences_source_good = correspondences;
	}
	else
	{
		std::vector<int> inliers;
		// Get the inliers
		sac.getInliers (inliers);
		int c=0;
		for (size_t i = 0; i < correspondences.size (); ++i)
		{
			if(i==inliers[c])
			{
				correspondences_source_good.push_back(correspondences[inliers[c]]);
				c++;
			}
		}

	}

	if(showDisplay)
	{
		ROS_INFO("number of correspondences  after ransac %d",correspondences_source_good.size());
	}
	ransac_inliers=correspondences_source_good.size();

	//Check whether we need next keyframe
	if(ransac_inliers<min_keyframe_inlier)
		next_keyframe=true;
	else
		next_keyframe=false;

	// Check whether we have enough correspondences
	if (ransac_inliers < min_inliers)
	{
		if(showDisplay)
			ROS_ERROR ("[pcl::::computeTransformation] Not enough correspondences found. Relax your threshold parameters.");
		compute_transform_success=false;
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
		if(showDisplay)
		{
			std::cout<<"transformation_"<<std::endl<<transformation_<<std::endl;
			std::cout<<"KeyframeDataVector.at(actual_keyframe).Transformation"<<std::endl<<KeyframeDataVector.at(actual_keyframe).Transformation<<std::endl;
			std::cout<<"actual keyframe"<<actual_keyframe<<std::endl;
		}

		transformOld=KeyframeDataVector.at(actual_keyframe).Transformation*transformation_;



	}
	else
	{

		if(transformation_at_least_twice_computed)
		{
			frametrans=KeyframeDataVector.at(actual_keyframe).Transformation*transformation_*previous_transformation.inverse()*transformation_;
			translation=frametrans.block<3,1>(0,3);
			if(showDisplay)
				std::cout<<"frametrans"<<frametrans<<std::endl;
			transformOld=frametrans;
		}
		else
			transformOld=Eigen::Matrix4f::Identity();


	}
	if(take_vicon)
	{
		next_keyframe=true;
		transformOld=vicontransform;
		if(showDisplay)
			std::cout<<"vicontransform:"<<std::endl<<vicontransform<<std::endl;

		take_vicon=false;
	}

	if(showDisplay)
		std::cout<<"transformold"<<std::endl<<transformOld<<std::endl;
	correspondences_inliers=correspondences_source_good;

	if(next_keyframe)
	{
		FrameData[counter].edges_to.push_back(actual_keyframe);
		FrameData[counter].Transformation_to.push_back(transformation_);
	}

}

void LOC::swap()
{

	if(showDisplay)
		std::cout<<"swapping"<<std::endl;

	FrameData[0].Descriptor=FrameData[1].Descriptor;
	FrameData[0].Points=FrameData[1].Points;
	FrameData[1].matches_backward=matches_popcount;
	FrameData[1].correspondences_backward=correspondences_inliers;
	if(showDisplay)
		std::cout<<"keyframdatavector"<<std::endl;
	KeyframeDataVector.push_back(FrameData[1]);
	if(showDisplay)
		std::cout<<"after"<<std::endl;
	actual_keyframe=KeyframeDataVector.size()-1;

	kpts[0]=kpts[1];
	dtors[0]=dtors[1];

	kinectCloud[0]=kinectCloud[1];
	cvCopy(cv_image[1],cv_image[0]);

	FrameData[0].Transformation=FrameData[1].Transformation;
	FrameData[0].edges_to=FrameData[1].edges_to;
	FrameData[0].Transformation_to=FrameData[0].Transformation_to;
	FrameData[1].edges_to.clear();
	FrameData[1].Transformation_to.clear();

}

void LOC::swapSingleFrame()
{

	if(showDisplay)
		std::cout<<"swapping"<<std::endl;

	FrameData[1].Descriptor=FrameData[0].Descriptor;
	FrameData[1].Points=FrameData[0].Points;

}

void LOC::computeTransformationToMap(struct FrameData& from, struct MapData &map, Eigen::Matrix4f &transform,int &inlier)
{
	if(showDisplay)
		std::cout<<"in computetransform to map"<<std::endl;
	Eigen::Vector3f tmp_dist;
	vector<cv::DMatch> matches;
	std::vector<int> corr_Keyframe;
	std::vector<int> corr_Keyframe_inliers;
	std::vector<int> corr_Keyframe_inliers_tmp;
	std::vector<int> corr_Keyframe_outliers;
	PointCloud Feature_Keyframe[2];
	Feature_Keyframe[0].header.frame_id="/pgraph";
	Feature_Keyframe[1].header.frame_id="/pgraph";


	//	std::cout<<"mapdescripotr:"<<map.Descriptor<<std::endl;
	if(showDisplay)
		start_match=clock();
	matchFeature(from.Descriptor,map.Descriptor,matches);
	if(showDisplay)
	{
		end_match=clock();
		std::cout<<"matching features: "<<matches.size()<<std::endl;
		std::cout<<"and it takes"<<(double(end_match)-double(start_match))/double(CLOCKS_PER_SEC);


	}

	for (size_t i = 0; i < matches.size(); i++)
	{
		Feature_Keyframe[0].push_back(from.Points.points[matches[i].queryIdx]);
		corr_Keyframe.push_back(i);
		Feature_Keyframe[1].push_back(map.Points.points[matches[i].trainIdx]);

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

	}
	if(showDisplay)
		std::cout<<"ransac inliers:"<<corr_Keyframe_inliers.size()<<std::endl;
	inlier=corr_Keyframe_inliers.size();


	pcl::estimateRigidTransformationSVD(Feature_Keyframe[0],corr_Keyframe_inliers,Feature_Keyframe[1],corr_Keyframe_inliers,transform);
	if(showDisplay)
		std::cout<<"transform:"<<transform<<std::endl;


}

void LOC::computeTransformationBetweenFrameData(struct FrameData& from, struct FrameData &map, Eigen::Matrix4f &transform, int &inlier)
{
	if(showDisplay)
		std::cout<<"in computetransform between framedata"<<std::endl;
	Eigen::Vector3f tmp_dist;
	vector<cv::DMatch> matches;
	std::vector<int> corr_Keyframe;
	std::vector<int> corr_Keyframe_inliers;
	std::vector<int> corr_Keyframe_inliers_tmp;
	std::vector<int> corr_Keyframe_outliers;
	PointCloud Feature_Keyframe[2];
	Feature_Keyframe[0].header.frame_id="/pgraph";
	Feature_Keyframe[1].header.frame_id="/pgraph";


	//	std::cout<<"mapdescripotr:"<<map.Descriptor<<std::endl;
	if(showDisplay)
		start_match=clock();
	matchFeature(from.Descriptor,map.Descriptor,matches);
	if(showDisplay)
	{
		end_match=clock();
		std::cout<<"matching features: "<<matches.size()<<std::endl;
		std::cout<<"and it takes"<<(double(end_match)-double(start_match))/double(CLOCKS_PER_SEC);


	}

	for (size_t i = 0; i < matches.size(); i++)
	{
		Feature_Keyframe[0].push_back(from.Points.points[matches[i].queryIdx]);
		corr_Keyframe.push_back(i);
		Feature_Keyframe[1].push_back(map.Points.points[matches[i].trainIdx]);

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

	}
	if(showDisplay)
		std::cout<<"ransac inliers:"<<corr_Keyframe_inliers.size()<<std::endl;
	inlier=corr_Keyframe_inliers.size();


	pcl::estimateRigidTransformationSVD(Feature_Keyframe[0],corr_Keyframe_inliers,Feature_Keyframe[1],corr_Keyframe_inliers,transform);
	if(showDisplay)
		std::cout<<"transform:"<<transform<<std::endl;
}

void LOC::computePointsWithin2Circles(pcl::KdTreeFLANN<Point> &tree, float zdist1, float zdist2, float radius1, float radius2, Eigen::Matrix4f estimated_trans, struct MapData &mapin, struct MapData &mapout,PointCloud &searchpoints)
{

	Eigen::Vector3f dist_point_vec(0,0,zdist1);
	Eigen::Matrix4f dist_point_mat=Eigen::Matrix4f::Identity();
	dist_point_mat.block<3,1>(0,3)=dist_point_vec;
	dist_point_mat=estimated_trans*dist_point_mat;
	//				std::cout<<"distpointmat"<<std::endl<<dist_point_mat<<std::endl;

	Point searchpoint;
	dist_point_vec=dist_point_mat.block<3,1>(0,3);
	searchpoint.x=dist_point_vec[0];
	searchpoint.y=dist_point_vec[1];
	searchpoint.z=dist_point_vec[2];

	searchpoints.points.push_back(searchpoint);

	std::vector<int> indices_first;
	std::vector<float> distances_first;
	tree.radiusSearch(searchpoint,radius1,indices_first,distances_first);

	dist_point_vec[0]=0;
	dist_point_vec[1]=0;
	dist_point_vec[2]=zdist2;

	dist_point_mat=Eigen::Matrix4f::Identity();

	dist_point_mat.block<3,1>(0,3)=dist_point_vec;

	dist_point_mat=estimated_trans*dist_point_mat;

	dist_point_vec=dist_point_mat.block<3,1>(0,3);
	searchpoint.x=dist_point_vec[0];
	searchpoint.y=dist_point_vec[1];
	searchpoint.z=dist_point_vec[2];

	searchpoints.points.push_back(searchpoint);

	std::vector<int> indices_second;
	std::vector<float> distances_second;
	tree.radiusSearch(searchpoint,radius2,indices_second,distances_second);

	std::vector<int> indices_total;
	indices_total=indices_first;
	indices_total.insert(indices_total.end(),indices_second.begin(),indices_second.end());

	std::sort(indices_total.begin(), indices_total.end());
	indices_total.erase(std::unique(indices_total.begin(), indices_total.end()), indices_total.end());

	//				std::cout<<"indices_first"<<std::endl;
	//				for(uint i=0;i<indices_first.size();i++)
		//					std::cout<<indices_first.at(i)<<std::endl;
	//				std::cout<<"indices_second"<<std::endl;
	//				for(uint i=0;i<indices_second.size();i++)
	//					std::cout<<indices_second.at(i)<<std::endl;
	//				std::cout<<"indices_total"<<std::endl;
	//				for(uint i=0;i<indices_total.size();i++)
	//					std::cout<<indices_total.at(i)<<std::endl;

	struct MapData tmp_map;
	tmp_map.Descriptor.flags=mapin.Descriptor.flags;
	tmp_map.Descriptor.dims=mapin.Descriptor.dims;
	tmp_map.Descriptor.cols=mapin.Descriptor.cols;
	tmp_map.Descriptor.step[0]=mapin.Descriptor.step[0];
	tmp_map.Descriptor.step[1]=mapin.Descriptor.step[1];

	for(uint i=0;i<indices_total.size();i++)
	{
		tmp_map.Points.push_back(mapin.Points.points.at(indices_total.at(i)));
		tmp_map.Descriptor.push_back(mapin.Descriptor.row(indices_total.at(i)));
	}

	mapout=tmp_map;

}


